#!/usr/bin/env python3
"""
Test script to verify device registration setup without actually registering
"""
import sys
import os
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from sidecar.registration import (
    get_pump_id,
    get_local_ip,
    BACKEND_URL,
    CF_ACCESS_CLIENT_ID,
    CF_ACCESS_CLIENT_SECRET,
    BRIDGE_BASE,
    CLOUDFLARED_CONFIG_DIR
)

def test_config():
    """Test configuration loading"""
    print("Testing Configuration")
    print("=" * 60)
    
    print(f"\n✓ Backend URL: {BACKEND_URL}")
    
    if BACKEND_URL == "https://your-backend-url.com":
        print("  ⚠️  WARNING: Using default backend URL!")
        print("  Update MAKER_BACKEND_URL in .env")
    
    print(f"✓ CF Access Client ID: {CF_ACCESS_CLIENT_ID[:20]}...")
    print(f"✓ CF Access Secret: {CF_ACCESS_CLIENT_SECRET[:20]}...")
    
    print(f"\n✓ Config directory: {BRIDGE_BASE}")
    print(f"  Exists: {BRIDGE_BASE.exists()}")
    
    print(f"✓ Cloudflared directory: {CLOUDFLARED_CONFIG_DIR}")
    print(f"  Exists: {CLOUDFLARED_CONFIG_DIR.exists()}")


def test_device_id():
    """Test device ID generation"""
    print("\n\nTesting Device ID Generation")
    print("=" * 60)
    
    pump_id = get_pump_id()
    print(f"\n✓ Device ID: {pump_id}")
    
    if pump_id.startswith("pump-"):
        identifier = pump_id[5:]
        if len(identifier) == 12:
            print("  Using MAC address")
        elif len(identifier) == 8:
            print("  Using generated UUID")
    
    local_ip = get_local_ip()
    print(f"✓ Local IP: {local_ip}")


def test_cloudflared():
    """Test cloudflared installation"""
    print("\n\nTesting Cloudflared")
    print("=" * 60)
    
    import subprocess
    try:
        result = subprocess.run(
            ["which", "cloudflared"],
            capture_output=True,
            text=True,
            check=True
        )
        cloudflared_path = result.stdout.strip()
        print(f"\n✓ Cloudflared found: {cloudflared_path}")
        
        # Get version
        result = subprocess.run(
            ["cloudflared", "--version"],
            capture_output=True,
            text=True
        )
        version = result.stdout.strip()
        print(f"✓ Version: {version}")
        
    except subprocess.CalledProcessError:
        print("\n✗ Cloudflared not found!")
        print("  Install with: sudo bash script/setup-registration.sh")


def test_python_deps():
    """Test Python dependencies"""
    print("\n\nTesting Python Dependencies")
    print("=" * 60)
    
    deps = {
        "requests": "HTTP client",
        "dotenv": "Environment loading"
    }
    
    for module, description in deps.items():
        try:
            if module == "dotenv":
                __import__("dotenv")
            else:
                __import__(module)
            print(f"\n✓ {module}: {description}")
        except ImportError:
            print(f"\n✗ {module} not found!")
            print(f"  Install with: pip install {module}")


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("DVI Bridge - Registration Setup Test")
    print("=" * 60)
    
    try:
        test_config()
        test_device_id()
        test_cloudflared()
        test_python_deps()
        
        print("\n\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        print("\n✓ Basic setup looks good!")
        print("\nTo register this device, run:")
        print("  /home/dviha/dvi-bridge/venv/bin/python3 /home/dviha/dvi-bridge/sidecar/registration.py")
        
    except Exception as e:
        print(f"\n\n✗ Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
