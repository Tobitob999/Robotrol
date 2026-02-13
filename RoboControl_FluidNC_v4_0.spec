# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

a = Analysis(
    ['RoboControl_FluidNC_v4_9.py'],
    pathex=[],
    binaries=[],
    datas=[
        # App-Module / Assets
        ('tcp_world_kinematics_frame.py', '.'),
        ('fluidnc_updater_v2.py', '.'),
        ('gamepad_block_v3.py', '.'),
        ('robosim_visualizer_v90.py', '.'),
        ('camera_capturev_v1_1.py', '.'),
        ('board_pose_v1.py', '.'),
        ('tcp_pose_module_v3.py', '.'),
    ],
    hiddenimports=[
        'serial',
        'serial.tools.list_ports',
        'pygame',
        'psutil',
        'numpy',
        'cv2',
        'PIL.Image',
        'PIL.ImageTk',
        'matplotlib.backends.backend_tkagg',
        'tkinter',
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    name='RoboControl_FluidNC_v4_9',
    icon=None,
    console=False,                 # 🚫 kein schwarzes Fenster
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,                      # Kompression aktiv
    upx_exclude=[],
    runtime_tmpdir=None,
    disable_windowed_traceback=False,
    onefile=True,                  # ✅ OneFile aktiviert
)
