"""
Setup Script for AI Employee - Gold Tier
Initializes the system and checks all dependencies
"""
import sys
import os
from pathlib import Path
import subprocess

def check_python_version():
    """Check Python version is 3.13+"""
    print("Checking Python version...")
    version = sys.version_info
    if version.major < 3 or (version.major == 3 and version.minor < 13):
        print(f"❌ Python 3.13+ required, found {version.major}.{version.minor}")
        return False
    print(f"✅ Python {version.major}.{version.minor}.{version.micro}")
    return True

def check_dependencies():
    """Check if required packages are installed"""
    print("\nChecking dependencies...")
    required = [
        'dotenv',
        'google.auth',
        'googleapiclient',
        'playwright',
        'watchdog',
        'markdown'
    ]

    missing = []
    for package in required:
        try:
            __import__(package.replace('.', '_'))
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package} not found")
            missing.append(package)

    if missing:
        print(f"\n⚠️  Missing packages: {', '.join(missing)}")
        print("Run: pip install -r requirements.txt")
        return False

    return True

def check_claude_code():
    """Check if Claude Code is installed"""
    print("\nChecking Claude Code...")
    try:
        result = subprocess.run(['claude', '--version'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ Claude Code installed")
            return True
    except:
        pass

    print("❌ Claude Code not found")
    print("Install from: https://claude.com/product/claude-code")
    return False

def check_obsidian():
    """Check if Obsidian is installed"""
    print("\nChecking Obsidian...")
    # Can't easily check if Obsidian is installed, just inform user
    print("ℹ️  Please ensure Obsidian is installed")
    print("Download from: https://obsidian.md/download")
    return True

def create_env_file():
    """Create .env file from template if it doesn't exist"""
    print("\nChecking environment configuration...")
    env_file = Path('.env')
    env_example = Path('.env.example')

    if env_file.exists():
        print("✅ .env file exists")
        return True

    if env_example.exists():
        print("⚠️  .env file not found, creating from template...")
        env_file.write_text(env_example.read_text())
        print("✅ Created .env file")
        print("⚠️  Please edit .env and add your credentials")
        return False

    print("❌ .env.example not found")
    return False

def check_vault_structure():
    """Check if vault structure exists"""
    print("\nChecking vault structure...")
    vault_path = Path('AI_Employee_Vault/vault')

    required_folders = [
        'Inbox', 'Needs_Action', 'In_Progress', 'Done',
        'Pending_Approval', 'Approved', 'Rejected',
        'Plans', 'Logs', 'Briefings', 'Accounting'
    ]

    all_exist = True
    for folder in required_folders:
        folder_path = vault_path / folder
        if folder_path.exists():
            print(f"✅ {folder}")
        else:
            print(f"❌ {folder} missing")
            folder_path.mkdir(parents=True, exist_ok=True)
            print(f"   Created {folder}")
            all_exist = False

    return True

def check_drop_folder():
    """Check if Drop_Folder exists"""
    print("\nChecking Drop_Folder...")
    drop_folder = Path('Drop_Folder')

    if drop_folder.exists():
        print("✅ Drop_Folder exists")
        return True

    print("⚠️  Creating Drop_Folder...")
    drop_folder.mkdir(parents=True, exist_ok=True)
    print("✅ Drop_Folder created")
    return True

def install_playwright():
    """Install Playwright browsers"""
    print("\nInstalling Playwright browsers...")
    try:
        result = subprocess.run(['playwright', 'install', 'chromium'],
                              capture_output=True, text=True, timeout=120)
        if result.returncode == 0:
            print("✅ Playwright browsers installed")
            return True
        else:
            print("⚠️  Playwright browser installation had issues")
            return False
    except Exception as e:
        print(f"⚠️  Could not install Playwright browsers: {e}")
        return False

def setup_gmail_credentials():
    """Guide user through Gmail API setup"""
    print("\n" + "="*60)
    print("Gmail API Setup")
    print("="*60)
    print("\nTo use Gmail integration, you need:")
    print("1. Go to: https://console.cloud.google.com/")
    print("2. Create a new project")
    print("3. Enable Gmail API")
    print("4. Create OAuth 2.0 credentials")
    print("5. Download credentials.json")
    print("6. Place credentials.json in AI_Employee_Vault/")
    print("\nPress Enter when done (or skip)...")
    input()

def setup_odoo():
    """Guide user through Odoo setup"""
    print("\n" + "="*60)
    print("Odoo Accounting Setup (Optional)")
    print("="*60)
    print("\nTo use Odoo integration:")
    print("1. Install Odoo Community Edition 19+")
    print("2. Create a database")
    print("3. Configure credentials in .env file")
    print("4. Test connection with: python -c 'import xmlrpc.client; ...'")
    print("\nPress Enter to continue...")
    input()

def main():
    """Main setup function"""
    print("="*60)
    print("AI Employee - Gold Tier Setup")
    print("="*60)

    checks = [
        ("Python Version", check_python_version),
        ("Dependencies", check_dependencies),
        ("Claude Code", check_claude_code),
        ("Obsidian", check_obsidian),
        ("Environment Config", create_env_file),
        ("Vault Structure", check_vault_structure),
        ("Drop Folder", check_drop_folder),
    ]

    results = {}
    for name, check_func in checks:
        results[name] = check_func()

    # Optional setups
    print("\n" + "="*60)
    print("Optional Components")
    print("="*60)

    response = input("\nInstall Playwright browsers? (y/n): ")
    if response.lower() == 'y':
        install_playwright()

    response = input("\nSetup Gmail API? (y/n): ")
    if response.lower() == 'y':
        setup_gmail_credentials()

    response = input("\nSetup Odoo? (y/n): ")
    if response.lower() == 'y':
        setup_odoo()

    # Summary
    print("\n" + "="*60)
    print("Setup Summary")
    print("="*60)

    all_passed = all(results.values())

    for name, passed in results.items():
        status = "✅" if passed else "❌"
        print(f"{status} {name}")

    if all_passed:
        print("\n✅ Setup complete! You're ready to start.")
        print("\nNext steps:")
        print("1. Edit .env with your credentials")
        print("2. Open vault in Obsidian: AI_Employee_Vault/vault")
        print("3. Start orchestrator: python AI_Employee_Vault/orchestrator.py AI_Employee_Vault/vault")
        print("4. Drop files into Drop_Folder to test")
    else:
        print("\n⚠️  Some checks failed. Please resolve issues above.")
        print("Run this script again after fixing issues.")

    print("\n" + "="*60)

if __name__ == '__main__':
    main()
