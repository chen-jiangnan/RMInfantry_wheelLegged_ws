#pragma once
#include <functional>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cstdint>
// ================================================================
//  JFSMode
// ================================================================
enum class JFSMode {
    ZEROTAU,
    CALI,
    DAMPING,
    RESET,
    READY,
    NONE=0xFF
};
// 模式值到触发字符串的映射
static const std::unordered_map<uint8_t, std::string> kFSMTriggerMap = {
    {0, "zerotau"},
    {1, "cali"},
    {2, "damping"},
    {3, "reset"},
    {4, "ready"},
    {0xFF, ""},// ← NONE，底层收到后 find 不到有效触发，不做任何切换
};
// JFSM 配置用于config_manager
struct CaliConfig {
    int   direction        = 1;
    float velocity         = 0.5f;
    float torque_threshold = 8.0f;
    float timeout          = 5.0f;
};

struct ResetConfig {
    struct state{
        float l0   = 0.395f;
        float phi0 = 2.416f;
    };
    state state_up   = {0.395f, 2.416f};
    state state_down = {0.395f, 2.416f};

    float leg_l0_step   = 0.005f;
    float leg_phi0_step = 0.001f;
};

struct JFSMConfig {
    bool require_cali = true;   // false → 跳过 CALI，event_.cali 初始为 true
    CaliConfig  cali;
    ResetConfig reset;
};

// ================================================================
//  FSMEvent — 外部条件，由控制节点写入
// ================================================================
struct FSMEvent {
    bool error = false;   // 错误标志（过流/过温/通信丢失等）
    bool cali  = false;   // 校准完成标志
    // 后续扩展：
    // bool fall_detected = false;
    // bool low_battery   = false;
};

// ================================================================
//  Transition — 转移表项
// ================================================================
struct Transition {
    JFSMode     from;
    JFSMode     to;
    std::string trigger;              // 触发事件名（字符串，便于 topic 传入）
    std::function<bool()> guard;      // 守卫条件，nullptr 表示无条件
    std::string deny_msg;             // 守卫不满足时的提示
};

// ================================================================
//  JointFSM
//
//  生命周期：
//    trigger("xxx") / checkAutoTransitions()
//      └─ doTransit(to)
//           ├─ callExit(current)   ← onExit 回调执行一次（清理）
//           ├─ current = to
//           └─ callEnter(to)       ← onEnter 回调执行一次（初始化）
//
//    之后 controlLoop 的 switch(fsm->current()) 持续执行每帧控制
//    onEnter/onExit 不会再次触发，直到下一次状态切换
// ================================================================
class JointFSM {
public:
    using Callback = std::function<void()>;

    explicit JointFSM(FSMEvent& ev)
        : ev_(ev), current_(JFSMode::ZEROTAU)
    {
        buildTransitions();
        // 初始进入 ZEROTAU，触发 onEnter
        callEnter(current_);
    }

    // ── 用户触发（按键 / topic）──────────────────────────────
    // trigger("damping") / trigger("ready") / trigger("zerotau") 等
    bool trigger(const std::string& event_name) {
        // 错误时只允许切到 ZEROTAU
        if (ev_.error && event_name != "zerotau") {
            std::cout << "[FSM][ERROR] 存在错误，只能切换到 ZEROTAU\n";
            if (current_ != JFSMode::ZEROTAU)
                doTransit(JFSMode::ZEROTAU);
            return false;
        }

        for (auto& t : transitions_) {
            if (t.from    != current_)    continue;
            if (t.trigger != event_name)  continue;

            if (t.guard && !t.guard()) {
                std::cout << "[FSM][WARN] " << t.deny_msg << "\n";
                return false;
            }
            return doTransit(t.to);
        }
        
        if ( stateName(current_) != event_name){
            std::cout << "[FSM] 当前状态 [" << stateName(current_)
            << "] 不支持触发: " << event_name << "\n";
        }
        return false;
    }

    // ── 自动检测（在 controlLoop 每帧调用）─────────────────
    void checkAutoTransitions() {
        if (ev_.error && current_ != JFSMode::ZEROTAU) {
            std::cout << "[FSM][AUTO] 检测到错误，强制切换到 ZEROTAU\n";
            doTransit(JFSMode::ZEROTAU);
        }
        // 后续可扩展：
        // if (ev_.fall_detected && current_ == JFSMode::READY)
        //     doTransit(JFSMode::DAMPING);
    }

    // ── 注册回调 ────────────────────────────────────────────
    // onEnter：状态切入时执行一次（初始化、清空控制器等）
    void onEnter(JFSMode s, Callback cb) { enter_cbs_[s] = std::move(cb); }
    // onExit：状态切出时执行一次（清理、保存状态等）
    void onExit (JFSMode s, Callback cb) { exit_cbs_[s]  = std::move(cb); }

    JFSMode current() const { return current_; }

    static std::string stateName(JFSMode s) {
        switch (s) {
            case JFSMode::ZEROTAU: return "zerotau";
            case JFSMode::CALI:    return "cali";
            case JFSMode::DAMPING: return "damping";
            case JFSMode::RESET:   return "reset";
            case JFSMode::READY:   return "ready";
        }
        return "UNKNOWN";
    }

private:
    bool doTransit(JFSMode to) {
        callExit(current_);                              // 1. 退出旧状态（一次）
        std::cout << "[FSM] " << stateName(current_)
                  << " → " << stateName(to) << "\n";
        current_ = to;
        callEnter(current_);                             // 2. 进入新状态（一次）
        return true;
    }

    void callEnter(JFSMode s) {
        auto it = enter_cbs_.find(s);
        if (it != enter_cbs_.end()) it->second();
    }
    void callExit(JFSMode s) {
        auto it = exit_cbs_.find(s);
        if (it != exit_cbs_.end()) it->second();
    }

    void buildTransitions() {
        // ── 转移表：所有合法转移集中在此 ──
        // 新增状态或转移只需在这里加行，不需要改其他地方
        transitions_ = {
            // ZEROTAU 可以去
            {JFSMode::ZEROTAU, JFSMode::DAMPING, "damping", nullptr,              ""},
            {JFSMode::ZEROTAU, JFSMode::CALI,    "cali",    nullptr,              ""},
            {JFSMode::ZEROTAU, JFSMode::READY,   "ready",   [&]{ return ev_.cali; }, "请先完成校准再进入 READY"},
            {JFSMode::ZEROTAU, JFSMode::RESET,   "reset",   [&]{ return ev_.cali; }, "请先完成校准再进入 RESET"},

            // CALI 可以去
            {JFSMode::CALI,    JFSMode::ZEROTAU, "zerotau", nullptr,              ""},
            {JFSMode::CALI,    JFSMode::READY,   "ready",   [&]{ return ev_.cali; }, "等待校准完成"},

            // DAMPING 可以去
            {JFSMode::DAMPING, JFSMode::ZEROTAU, "zerotau", nullptr,              ""},
            {JFSMode::DAMPING, JFSMode::CALI,    "cali",    nullptr,              ""},
            {JFSMode::DAMPING, JFSMode::RESET,   "reset",   [&]{ return ev_.cali; }, "请先完成校准再进入 RESET"},

            // READY 可以去
            {JFSMode::READY,   JFSMode::ZEROTAU, "zerotau", nullptr,              ""},
            {JFSMode::READY,   JFSMode::DAMPING, "damping", nullptr,              ""},

            // RESET 可以去
            {JFSMode::RESET,   JFSMode::DAMPING, "damping", nullptr,              ""},
            {JFSMode::RESET,   JFSMode::READY,   "ready",   [&]{ return ev_.cali; }, "请先完成校准再进入 READY"},
            {JFSMode::RESET,   JFSMode::ZEROTAU, "zerotau", nullptr,              ""},
        };
    }

    FSMEvent&                                          ev_;
    JFSMode                                            current_;
    std::vector<Transition>                            transitions_;
    std::unordered_map<JFSMode, Callback>              enter_cbs_;
    std::unordered_map<JFSMode, Callback>              exit_cbs_;
};