"""Extract the embedded AutoKart RC page into a browser-openable preview."""

import ast
from pathlib import Path


root = Path(__file__).resolve().parents[1]
firmware = root / "bench/autokart_rc_simple_encoders.py"
preview = root / "docs/autokart_rc_ui_preview.html"
tree = ast.parse(firmware.read_text(encoding="utf-8"))
html = next(
    ast.literal_eval(node.value)
    for node in tree.body
    if isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == "HTML" for target in node.targets)
)
preview.write_text(html + "\n", encoding="utf-8")
print(preview)
