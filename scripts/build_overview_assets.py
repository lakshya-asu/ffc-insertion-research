"""Create smaller presentation copies of existing Isaac renders; preserve originals."""
import hashlib
import json
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/overview/images'
SOURCES = {
    'cell': 'docs/live/mount/03-cell.webp',
    'placement': 'docs/live/mount/03-placement.webp',
    'macro': 'docs/live/mount/03-macro.webp',
}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = []
    for name, relative in SOURCES.items():
        source = ROOT / relative
        for width in (640, 960):
            with Image.open(source) as image:
                image.thumbnail((width, width), Image.Resampling.LANCZOS)
                target = OUT / f'{name}-{width}.webp'
                image.save(target, quality=85, method=6)
                manifest.append({
                    'source': relative,
                    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                    'output': target.name,
                    'output_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                    'size': list(image.size),
                    'purpose': 'Presentation resize only; original evidence remains unchanged',
                })
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
