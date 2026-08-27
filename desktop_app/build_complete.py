# build_complete.py
import os
import sys
import subprocess
import shutil

# Ensure UTF-8 output on Windows consoles
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

from PIL import Image

def convert_png_to_ico(input_file, output_file):
    """Convert PNG to ICO format with multiple sizes"""
    try:
        if not os.path.exists(input_file):
            print(f"   ⚠️  {input_file} not found")
            return False
        
        print(f"   Converting {input_file} to ICO...")
        img = Image.open(input_file)
        
        # Standard Windows icon sizes
        sizes = [(256, 256), (128, 128), (96, 96), (64, 64), (48, 48), (32, 32), (24, 24), (16, 16)]
        
        # Create icon with multiple sizes
        icons = []
        for size in sizes:
            resized = img.resize(size, Image.Resampling.LANCZOS)
            if resized.mode != 'RGBA':
                resized = resized.convert('RGBA')
            icons.append(resized)
        
        # Save as ICO
        icons[0].save(output_file, format='ICO', sizes=sizes, append_images=icons[1:])
        print(f"   ✅ Icon converted: {output_file}")
        return True
        
    except Exception as e:
        print(f"   ❌ Error converting icon: {e}")
        return False

def run_command(cmd, capture_output=False):
    """Run a command and print output"""
    print(f"\n> {' '.join(cmd)}")
    if capture_output:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        return result.returncode, result.stdout, result.stderr
    else:
        result = subprocess.run(cmd)
        return result.returncode, "", ""

def find_main_script():
    """Find the main Python script to build"""
    # Possible script names
    possible_names = ['UIMODE.py', 'Sess.py', 'bms_logger.py', 'main.py', 'data_logger.py', 'logger.py']
    
    for name in possible_names:
        if os.path.exists(name):
            print(f"   Found main script: {name}")
            return name
    
    # If no script found, ask user
    print("   ⚠️  Could not find main script automatically")
    print("   Available Python files:")
    py_files = [f for f in os.listdir('.') if f.endswith('.py') and f != 'build_complete.py' and f != 'convert_icon.py']
    for i, file in enumerate(py_files, 1):
        print(f"      {i}. {file}")
    
    if py_files:
        choice = input(f"   Select file (1-{len(py_files)}): ").strip()
        try:
            return py_files[int(choice) - 1]
        except:
            print("   Invalid choice, using UI_DATE.py as default")
            return "UI_DATE.py"
    
    return None

def main():
    print("=" * 60)
    print("BMS Data Logger - Build Script with Icon Integration")
    print("=" * 60)
    
    # Step 1: Check Python
    print("\n📌 Step 1: Checking Python...")
    python_version = run_command([sys.executable, '--version'], capture_output=True)
    if python_version[0] != 0:
        print("❌ Python not found!")
        return
    print("✅ Python found")
    
    # Step 2: Check and install Pillow if needed
    print("\n📌 Step 2: Checking Pillow for icon conversion...")
    try:
        import PIL
        print("✅ Pillow already installed")
    except ImportError:
        print("⚠️ Pillow not found. Installing...")
        install_cmd = [sys.executable, '-m', 'pip', 'install', 'pillow']
        if run_command(install_cmd)[0] != 0:
            print("❌ Failed to install Pillow!")
            return
        print("✅ Pillow installed successfully")
    
    # Step 3: Find main script
    print("\n📌 Step 3: Locating main script...")
    main_script = find_main_script()
    
    if not main_script:
        print("❌ No Python script found to build!")
        return
    
    print(f"✅ Using main script: {main_script}")
    
    # Step 4: Convert icon
    print("\n📌 Step 4: Processing icon...")
    icon_file = "DATALOGGER.png"
    ico_file = "app.ico"
    
    if os.path.exists(icon_file):
        print(f"✅ Found icon: {icon_file}")
        if convert_png_to_ico(icon_file, ico_file):
            print("✅ Icon ready for build")
        else:
            print("⚠️ Using default icon (no custom icon)")
            ico_file = None
    else:
        print("⚠️ DATALOGGER.png not found! Building without custom icon")
        ico_file = None
    
    # Step 5: Check PyInstaller
    print("\n📌 Step 5: Checking PyInstaller...")
    try:
        result = subprocess.run([sys.executable, '-m', 'PyInstaller', '--version'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ PyInstaller found: {result.stdout.strip()}")
        else:
            print("⚠️ PyInstaller not found. Installing...")
            install_cmd = [sys.executable, '-m', 'pip', 'install', 'pyinstaller']
            if run_command(install_cmd)[0] != 0:
                print("❌ Failed to install PyInstaller!")
                return
            print("✅ PyInstaller installed successfully")
    except Exception as e:
        print(f"❌ Error checking PyInstaller: {e}")
        return
    
    # Step 6: Clean previous builds
    print("\n📌 Step 6: Cleaning previous builds...")
    for folder in ['build', 'dist', '__pycache__']:
        if os.path.exists(folder):
            shutil.rmtree(folder)
            print(f"   Removed {folder}")
    
    for file in os.listdir('.'):
        if file.endswith('.spec'):
            os.remove(file)
            print(f"   Removed {file}")
    
    # Step 7: Build the executable
    print("\n📌 Step 7: Building executable...")
    build_cmd = [
        sys.executable, '-m', 'PyInstaller',
        '--onefile',
        '--windowed',
        '--name', 'BMS_Data_Logger',
        '--hidden-import', 'serial',
        '--hidden-import', 'serial.tools.list_ports',
        '--hidden-import', 'tkinter',
        '--hidden-import', 'zlib',
        '--hidden-import', 'threading',
        '--hidden-import', 'datetime',
        '--hidden-import', 'pathlib',
        '--clean',
        '--noconfirm',
        main_script  # Use the found main script
    ]
    
    # Add icon if available
    if ico_file and os.path.exists(ico_file):
        # Insert icon options at the correct position
        build_cmd.insert(5, f'--icon={ico_file}')
        build_cmd.insert(6, f'--add-data={ico_file};.')
        print(f"   Using custom icon: {ico_file}")
    
    print(f"\nRunning: {' '.join(build_cmd)}")
    result = subprocess.run(build_cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print("\n❌ Build failed!")
        if result.stderr:
            print("\nError output:")
            print(result.stderr)
        return
    
    print("\n✅ Build successful!")
    
    # Step 8: Check output
    exe_path = os.path.join('dist', 'BMS_Data_Logger.exe')
    if os.path.exists(exe_path):
        size_bytes = os.path.getsize(exe_path)
        size_mb = size_bytes / (1024 * 1024)
        print(f"\n📁 Executable: {os.path.abspath(exe_path)}")
        print(f"📊 File size: {size_mb:.2f} MB")
        
        # Create run script
        run_script = f"""@echo off
echo Starting BMS Data Logger...
echo.
start /B BMS_Data_Logger.exe
if errorlevel 1 (
    echo Error starting application
    pause
)
"""
        with open(os.path.join('dist', 'run_logger.bat'), 'w') as f:
            f.write(run_script)
        print("📝 Created run script: dist/run_logger.bat")
    else:
        print("⚠️ Executable not found in dist folder")
    
    # Step 9: Create README
    readme = """BMS Data Logger v3.0
====================

This is a standalone executable with custom icon.

To run:
1. Double-click BMS_Data_Logger.exe
   OR
2. Run run_logger.bat

Features:
- Connect to BMS device via USB
- List and download log files
- Filter files by date
- Console commands for advanced operations

System Requirements:
- Windows 7 or later
- USB port for device connection

Note: If you get a Windows Defender warning, click "More info" then "Run anyway"
"""
    
    with open(os.path.join('dist', 'README.txt'), 'w') as f:
        f.write(readme)
    print("📝 Created README: dist/README.txt")
    
    # Step 10: Copy icon to dist folder for reference
    if ico_file and os.path.exists(ico_file):
        shutil.copy2(ico_file, os.path.join('dist', 'app.ico'))
        print("📝 Copied icon to dist folder")
    
    print("\n" + "=" * 60)
    print("✅ Build completed successfully!")
    print("📁 Files are in the 'dist' folder")
    print("🎨 Custom icon integrated into executable")
    print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nBuild cancelled by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        if sys.stdin.isatty():
            try:
                input("\nPress Enter to exit...")
            except (EOFError, KeyboardInterrupt):
                pass