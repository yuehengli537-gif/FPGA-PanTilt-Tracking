"""Build and test an actual RAR source package; no hardware operations."""
import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rar', default=r'C:\Program Files\WinRAR\Rar.exe')
    parser.add_argument('--output-directory', type=Path, default=ROOT.parent / 'submission-artifacts')
    args = parser.parse_args()
    if not Path(args.rar).is_file():
        raise SystemExit('Install WinRAR/RAR or specify --rar. A ZIP renamed to RAR is not accepted.')
    spec = importlib.util.spec_from_file_location('submission_validator', ROOT / 'scripts/verify-submission.py')
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    validator.verify(write_manifest=False)
    manifest_path = ROOT / validator.MANIFEST
    if not manifest_path.exists():
        raise SystemExit('Run verify-submission.py --write-manifest before packaging.')
    saved = json.loads(manifest_path.read_text(encoding='utf-8'))
    files = list(validator.source_files())
    actual = {p.relative_to(ROOT).as_posix(): validator.digest(p) for p in files
              if p.relative_to(ROOT).as_posix() != validator.MANIFEST}
    if actual != {name: entry['sha256'] for name, entry in saved['files'].items()}:
        raise SystemExit('Manifest is stale. Re-validate, regenerate the manifest and commit changes first.')
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True, capture_output=True)
    worktree = subprocess.run(['git', 'status', '--porcelain'], cwd=ROOT, text=True, capture_output=True)
    if revision.returncode or worktree.returncode or worktree.stdout.strip():
        raise SystemExit('Commit the prepared source snapshot in a clean Git worktree before packaging.')
    source_revision = revision.stdout.strip()
    output = args.output_directory.resolve()
    if output == ROOT or output.is_relative_to(ROOT):
        raise SystemExit('Choose an output directory outside the source repository.')
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    archive = output / ('FPGA-PanTilt-Tracking_V8d_' + stamp + '.rar')
    if archive.exists():
        raise SystemExit('Archive already exists; refusing to replace it.')
    runtime = ROOT / 'reports/runtime/package'
    runtime.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='stage_', dir=runtime))
    package_root = stage / 'FPGA-PanTilt-Tracking'
    for source in files:
        target = package_root / source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        rel = source.relative_to(ROOT).as_posix()
        expected_hash = validator.digest(manifest_path) if rel == validator.MANIFEST else saved['files'][rel]['sha256']
        if validator.digest(target) != expected_hash:
            raise SystemExit('File changed while preparing archive: ' + rel)
    subprocess.run([args.rar, 'a', '-ma5', '-m5', '-r', '-idq', '-y', str(archive), package_root.name], cwd=stage, check=True)
    subprocess.run([args.rar, 't', '-idq', str(archive)], check=True)
    if not archive.read_bytes().startswith(b'Rar!\x1a\x07\x01\x00'):
        raise SystemExit('Unexpected archive signature')
    if archive.stat().st_size > 200_000_000:
        raise SystemExit('Archive exceeds the 200 MB submission limit')
    # RAR 6 treats -idq as suppressing even the bare listing, so omit it here.
    listing = subprocess.check_output([args.rar, 'lb', str(archive)], text=True, errors='replace')
    members = {line.replace('\\', '/').strip() for line in listing.splitlines() if line.strip()}
    expected = {package_root.name + '/' + p.relative_to(ROOT).as_posix() for p in files}
    # Directory entries are permitted; every regular source file must be present.
    if not expected.issubset(members):
        raise SystemExit('Archive file listing does not match the source files')
    sha = hashlib.sha256(archive.read_bytes()).hexdigest()
    record = {'archive': archive.name, 'bytes': archive.stat().st_size, 'sha256': sha,
              'source_commit': source_revision, 'source_worktree_clean': True,
              'staged_file_hashes_verified': True,
              'source_file_count': len(files), 'archive_format': 'RAR5', 'archive_test_passed': True,
              'scope': 'V8d source snapshot, not demonstration video, technical PDF or signed authorization'}
    archive.with_suffix('.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    archive.with_suffix('.sha256').write_text(sha + '  ' + archive.name + '\n', encoding='ascii')
    print(json.dumps(record, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
