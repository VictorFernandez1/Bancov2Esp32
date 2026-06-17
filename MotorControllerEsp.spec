# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec file for ESP32 Stepper Motor Controller GUI
Generates a single executable with windowed mode (no console)
"""

a = Analysis(
    ['MotorControllerEsp.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'bleak',
        'bleak.backends.winrt',
        'PySide6',
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludedimports=[],
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=None)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='MotorControllerEsp',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,  # No console window (windowed mode)
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    onefile=True,  # Single executable file
    icon=None,  # Optional: add custom icon path here (e.g., 'icon.ico')
)
