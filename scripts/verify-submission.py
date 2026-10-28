"""Validate the portable source snapshot without touching hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = 'docs/FILE_MANIFEST.json'
EXPECTED_FS = 'bec75258735164b027cc1262cc8fb56b59b3e12350bf323288fb8ffc85606456'
ARCHIVE = 'bitstreams/camera_uart_shape_diag_v8d_20261028.zip'


def source_files():
    for path in sorted(ROOT.rglob('*')):
        if not path.is_file():
            continue
        rel = path.relative_to(ROOT)
        if any(part in {'.git', '.tools', '.build', '__pycache__', 'submission-artifacts'} for part in rel.parts):
            continue
        name = rel.as_posix()
        if name.startswith(('reports/runtime/', 'algorithm_sim/build/')) or '/temp/' in name:
            continue
        if name.startswith('fpga/impl/') and name not in {
            'fpga/impl/OV5640_HDMI720P_process_config.json', 'fpga/impl/project_process_config.json'
        }:
            continue
        if path.suffix in {'.pyc', '.tmp', '.bak'} or path.name in {'Thumbs.db', '.DS_Store'}:
            continue
        yield path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(write_manifest=False):
    errors = []
    provenance = json.loads((ROOT / 'docs/SOURCE_PROVENANCE.json').read_text(encoding='utf-8-sig'))
    critical = {name: sha for name, sha in provenance['copied_files'].items()
                if name.startswith(('fpga/', 'algorithm_sim/', 'servo_uart/')) or name == ARCHIVE}
    for name, sha in critical.items():
        path = ROOT / name
        if not path.is_file() or digest(path) != sha:
            errors.append('Imported design/test file changed or missing: ' + name)
    project = ROOT / 'fpga/OV5640_HDMI720P.gprj'
    references = re.findall(r'<File\s+path="([^"]+)"', project.read_text(encoding='utf-8-sig'))
    for name in references:
        path = (project.parent / name).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file():
            errors.append('Missing/outside project dependency: ' + name)
    with zipfile.ZipFile(ROOT / ARCHIVE) as archive:
        candidates = [name for name in archive.namelist() if Path(name).name == 'OV5640_HDMI720P.fs']
        if len(candidates) != 1 or hashlib.sha256(archive.read(candidates[0])).hexdigest() != EXPECTED_FS:
            errors.append('Published bitstream hash mismatch')
    expected = {
        'Open-Control-Panel.cmd': 'scripts\u005cstart-servo-ui.ps1',
        'Download-V8d.cmd': 'scripts\u005cdownload-red-ball-contour.ps1',
        'Flash-V8d.cmd': 'scripts\u005cflash-red-ball-contour.ps1',
    }
    for name, target in expected.items():
        path = ROOT / name
        if not path.exists() or target not in path.read_text(encoding='utf-8-sig'):
            errors.append('Launcher mismatch: ' + name)
    for path in source_files():
        if path.suffix != '.md':
            continue
        text = path.read_text(encoding='utf-8-sig')
        for link in re.findall(r'\]\(([^)]+)\)', text):
            if '://' in link or link.startswith(('#', 'mailto:')):
                continue
            target = link.split('#', 1)[0].strip('<>')
            if target and not (path.parent / target).exists():
                errors.append('Broken documentation link: ' + path.relative_to(ROOT).as_posix() + ' -> ' + target)
    if errors:
        raise SystemExit('\n'.join(errors))
    files = {path.relative_to(ROOT).as_posix(): {'sha256': digest(path), 'bytes': path.stat().st_size}
             for path in source_files() if path.relative_to(ROOT).as_posix() != MANIFEST}
    record = {'firmware': 'V8d', 'bitstream_sha256': EXPECTED_FS,
              'upstream_source_commit': provenance['source_commit'],
              'file_count_excluding_manifest': len(files), 'files': files}
    if write_manifest:
        (ROOT / MANIFEST).write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'passed': True, 'project_references': len(references),
                      'unchanged_design_test_files': len(critical),
                      'file_count_excluding_manifest': len(files),
                      'total_bytes_excluding_manifest': sum(f['bytes'] for f in files.values()),
                      'manifest_written': write_manifest}, indent=2))
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-manifest', action='store_true')
    verify(parser.parse_args().write_manifest)
