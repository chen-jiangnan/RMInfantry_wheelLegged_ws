#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量覆盖 URDF 惯性属性（保留原始格式，支持多行属性格式）
- mass    : txt 中 质量=
- inertia : txt 中由重心决定的惯性张量（Lxx/Lxy/Lxz/Lyy/Lyz/Lzz）
            对应 URDF 的 ixx/ixy/ixz/iyy/iyz/izz

注意：origin xyz 不更新。
"""

import re
import shutil

# ── 配置 ─────────────────────────────────────────────────────
URDF_PATH = "wheelLegged_seriBot.urdf"
TXT_PATH  = "mass.txt"
OUT_PATH  = "wheelLegged_seriBot_updated.urdf"

# ── 备份 ─────────────────────────────────────────────────────
shutil.copy(URDF_PATH, URDF_PATH + ".bak")
print(f"[INFO] 已备份到 {URDF_PATH}.bak")

# ── 解析 txt ─────────────────────────────────────────────────
def parse_lmatrix_line(line, inertia):
    m = re.search(
        r"Lxx\s*=\s*([-0-9.eE]+)\s+"
        r"Lxy\s*=\s*([-0-9.eE]+)\s+"
        r"Lxz\s*=\s*([-0-9.eE]+)", line)
    if m:
        inertia["ixx"] = m.group(1)
        inertia["ixy"] = m.group(2)
        inertia["ixz"] = m.group(3)

    m = re.search(
        r"Lyx\s*=\s*[-0-9.eE]+\s+"
        r"Lyy\s*=\s*([-0-9.eE]+)\s+"
        r"Lyz\s*=\s*([-0-9.eE]+)", line)
    if m:
        inertia["iyy"] = m.group(1)
        inertia["iyz"] = m.group(2)

    m = re.search(
        r"Lzx\s*=\s*[-0-9.eE]+\s+"
        r"Lzy\s*=\s*[-0-9.eE]+\s+"
        r"Lzz\s*=\s*([-0-9.eE]+)", line)
    if m:
        inertia["izz"] = m.group(1)


def is_inertia_complete(inertia):
    return all(k in inertia for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))


def replace_attr_value(text, attr, new_val):
    """
    在 text 中把  attr="旧值"  替换为  attr="新值"
    支持 attr 与 = 之间有任意空白，支持多行。
    用 group(1)+new_val+group(2) 方式，不用 lookbehind。
    """
    pattern = re.compile(
        r'(\b' + re.escape(attr) + r'\s*=\s*")[^"]*(")',
        re.DOTALL
    )
    result, n = pattern.subn(lambda mo: mo.group(1) + new_val + mo.group(2),
                              text, count=1)
    return result, n


# ── 匹配 <inertia ... /> 块（单行/多行均可）────────────────────
INERTIA_BLOCK_RE = re.compile(
    r'<inertia\b[^>]*/>', re.DOTALL
)

# ── 读取 txt ──────────────────────────────────────────────────
link_data    = {}
current_link = None
in_com_tensor = False

with open(TXT_PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    stripped = lines[i].strip()

    if "的质量属性" in stripped:
        current_link = stripped.split()[0]
        link_data[current_link] = {"mass": None, "inertia": {}}
        in_com_tensor = False
        i += 1
        continue

    if current_link is None:
        i += 1
        continue

    data = link_data[current_link]

    m = re.search(r"^质量\s*=\s*([-0-9.eE]+)", stripped)
    if m:
        data["mass"] = m.group(1)
        i += 1
        continue

    if "由重心决定，并且对齐输出" in stripped:
        in_com_tensor = True
        data["inertia"] = {}
        i += 1
        continue

    if "由输出座标系决定" in stripped:
        in_com_tensor = False
        i += 1
        continue

    if in_com_tensor and not is_inertia_complete(data["inertia"]):
        parse_lmatrix_line(stripped, data["inertia"])
        if is_inertia_complete(data["inertia"]):
            in_com_tensor = False

    i += 1

# ── 打印解析结果 ──────────────────────────────────────────────
print(f"\n[INFO] txt 解析完成，共 {len(link_data)} 个 link：")
all_ok = True
for name, d in link_data.items():
    ok_mass    = d["mass"] is not None
    ok_inertia = is_inertia_complete(d["inertia"])
    status = "✅" if (ok_mass and ok_inertia) else "⚠️ "
    if not (ok_mass and ok_inertia):
        all_ok = False
    print(f"  {status} {name:35s}  mass={ok_mass}  inertia={ok_inertia}"
          + ("" if ok_inertia else f"  已有: {list(d['inertia'].keys())}"))

if not all_ok:
    print("\n[WARN] 部分 link 解析不完整，请检查 txt 格式")
else:
    print("\n[INFO] 所有 link 解析完整")

# ── 读取原始 URDF 文本 ────────────────────────────────────────
with open(URDF_PATH, "r", encoding="utf-8") as f:
    urdf_text = f.read()

# ── 逐 link 替换 ──────────────────────────────────────────────
LINK_BLOCK_RE = re.compile(
    r'(<link\b[^>]*\bname\s*=\s*"{name}"[^>]*>)(.*?)(</link>)',
    re.DOTALL
)

updated_count = 0
skip_count    = 0

for link_name, data in link_data.items():
    pattern = re.compile(
        r'(<link\b[^>]*\bname\s*=\s*"' + re.escape(link_name) + r'"[^>]*>)'
        r'(.*?)'
        r'(</link>)',
        re.DOTALL
    )
    m_link = pattern.search(urdf_text)
    if m_link is None:
        print(f"[WARN] URDF 中找不到 link: {link_name}")
        skip_count += 1
        continue

    link_open  = m_link.group(1)
    link_body  = m_link.group(2)
    link_close = m_link.group(3)
    link_start = m_link.start()
    link_end   = m_link.end()

    changed = False

    # ── 替换 mass ────────────────────────────────────────────
    if data["mass"] is not None:
        new_body, n = replace_attr_value(link_body, "value", data["mass"])
        if n:
            link_body = new_body
            changed = True
        else:
            print(f"[WARN] {link_name} mass value 属性未找到")

    # ── 替换 inertia 各分量 ───────────────────────────────────
    if is_inertia_complete(data["inertia"]):
        d = data["inertia"]

        # 先找到 <inertia .../> 块在 link_body 中的位置
        m_inertia = INERTIA_BLOCK_RE.search(link_body)
        if m_inertia is None:
            print(f"[WARN] {link_name} 未找到 <inertia .../> 块")
            skip_count += 1
            continue

        inertia_block = m_inertia.group(0)

        # 在块内逐一替换每个属性值
        for attr in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            inertia_block, _ = replace_attr_value(inertia_block, attr, d[attr])

        # 将替换后的块写回 link_body
        link_body = (link_body[:m_inertia.start()]
                     + inertia_block
                     + link_body[m_inertia.end():])
        changed = True
    else:
        print(f"[WARN] {link_name} 惯性张量不完整，跳过")
        skip_count += 1
        continue

    if changed:
        new_link_block = link_open + link_body + link_close
        urdf_text = urdf_text[:link_start] + new_link_block + urdf_text[link_end:]
        updated_count += 1

# ── 写出（保留原始格式）──────────────────────────────────────
with open(OUT_PATH, "w", encoding="utf-8") as f:
    f.write(urdf_text)

print(f"\n[INFO] 完成：更新 {updated_count} 个 link，跳过 {skip_count} 个")
print(f"[INFO] 输出文件：{OUT_PATH}")