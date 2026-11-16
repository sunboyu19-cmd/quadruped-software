"""One-time desktop copy, preserving all previous user projects."""
from pathlib import Path
import json
import shutil

source = Path(__file__).resolve().parents[1]
root = Path('E:/Users/32892/Desktop/机器狗/四轮足程序')
destination = root / '04_ESP32_QuadrupedBasic'
if destination.exists():
    raise SystemExit('Destination exists; refusing to overwrite: '+str(destination))
shutil.copytree(source, destination, ignore=shutil.ignore_patterns(
    '.pio', '.venv', '__pycache__', 'c_cpp_properties.json', 'ipch'))
workspace = root / '四轮足软件.code-workspace'
value = json.loads(workspace.read_text(encoding='utf-8'))
value['folders'].append({'name':'04 双ESP32 四轮四舵机基本运动', 'path':destination.name})
workspace.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
readme = root / 'README.md'
text = readme.read_text(encoding='utf-8')
text += '''

## 四轮＋四舵机整机基本运动

新增`04_ESP32_QuadrupedBasic`，面向物料清单里的两块普通ESP32-WROOM-32E（Micro-USB）。A控制前两轮和全部四舵机，B控制后两轮，UART连接。编号1左前、2右前、3左后、4右后，动力为6V。

先看该目录README完整接线和一根Micro-USB分次烧录步骤，之后双击目录内“打开控制面板.cmd”。需要先悬空校准、标定方向及舵机中位，再落地低速运动。新工程与前三个单项调试工程分别保留；整机仍需实物验证。
'''
readme.write_text(text, encoding='utf-8')
print(destination)
