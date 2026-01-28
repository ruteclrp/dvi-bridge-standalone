#!/bin/bash
# DVI Bridge - Authentication Setup Script
# Sets up or changes authentication credentials for tunnel access

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
AUTH_MODULE="$PROJECT_ROOT/src/sidecar/auth.py"
AUTH_CONFIG="/etc/dvi-bridge/auth.json"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   DVI Bridge - Authentication Setup       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════╝${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}❌ This script must be run as root${NC}"
    echo -e "${YELLOW}   Please run: sudo $0${NC}"
    exit 1
fi

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 is not installed${NC}"
    exit 1
fi

# Check if auth module exists
if [ ! -f "$AUTH_MODULE" ]; then
    echo -e "${RED}❌ Authentication module not found: $AUTH_MODULE${NC}"
    exit 1
fi

# Check if bcrypt is installed
if ! python3 -c "import bcrypt" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  bcrypt module not installed${NC}"
    echo -e "${BLUE}📦 Installing bcrypt...${NC}"
    
    cd "$PROJECT_ROOT/src/sidecar"
    pip3 install bcrypt || {
        echo -e "${RED}❌ Failed to install bcrypt${NC}"
        exit 1
    }
    echo -e "${GREEN}✅ bcrypt installed${NC}"
    echo ""
fi

# Display current status
echo -e "${BLUE}📋 Current Status:${NC}"
if [ -f "$AUTH_CONFIG" ]; then
    echo -e "   Config file: ${GREEN}Exists${NC}"
    CURRENT_USER=$(python3 -c "import json; print(json.load(open('$AUTH_CONFIG')).get('username', 'N/A'))" 2>/dev/null || echo "N/A")
    echo -e "   Current user: ${YELLOW}$CURRENT_USER${NC}"
else
    echo -e "   Config file: ${YELLOW}Not created yet${NC}"
    echo -e "   Default credentials will be created on first run"
fi
echo ""

# Menu
echo -e "${BLUE}What would you like to do?${NC}"
echo "  1) Change password"
echo "  2) View authentication status"
echo "  3) Enable/Disable authentication"
echo "  4) Exit"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo -e "${BLUE}🔐 Change Password${NC}"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        
        # Create config directory if it doesn't exist
        mkdir -p "$(dirname "$AUTH_CONFIG")"
        
        # Run the password change tool
        cd "$PROJECT_ROOT/src/sidecar"
        python3 "$AUTH_MODULE" --set-password
        
        # Set proper permissions
        chmod 600 "$AUTH_CONFIG"
        
        echo ""
        echo -e "${GREEN}✅ Password updated successfully!${NC}"
        echo ""
        echo -e "${YELLOW}📱 Update your mobile app settings with the new credentials${NC}"
        echo -e "${YELLOW}🔄 Restart webbridge service: sudo systemctl restart webbridge${NC}"
        ;;
        
    2)
        echo ""
        echo -e "${BLUE}📊 Authentication Status${NC}"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        
        if [ -f "$AUTH_CONFIG" ]; then
            python3 << EOF
import json
with open("$AUTH_CONFIG") as f:
    config = json.load(f)
    
print(f"Username:           {config.get('username', 'N/A')}")
print(f"Auth enabled:       {'Yes' if config.get('require_auth', True) else 'No'}")
print(f"Local auth:         {'Yes' if config.get('require_auth_local', False) else 'No'}")
print(f"Session timeout:    {config.get('session_timeout', 86400)} seconds ({config.get('session_timeout', 86400) // 3600} hours)")
EOF
        else
            echo -e "${YELLOW}⚠️  No configuration file found${NC}"
            echo -e "   Default credentials will be created on first run:"
            echo -e "   Username: ${GREEN}admin${NC}"
            echo -e "   Password: ${GREEN}dvibridge${NC}"
        fi
        ;;
        
    3)
        echo ""
        echo -e "${BLUE}🔒 Authentication Settings${NC}"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        
        if [ ! -f "$AUTH_CONFIG" ]; then
            echo -e "${RED}❌ No configuration file found${NC}"
            echo -e "   Run the webbridge service first to create default config"
            exit 1
        fi
        
        echo "  1) Enable authentication (recommended)"
        echo "  2) Disable authentication (NOT RECOMMENDED)"
        echo "  3) Require authentication on local network too"
        echo "  4) Don't require authentication on local network"
        echo ""
        read -p "Enter choice [1-4]: " auth_choice
        
        cd "$PROJECT_ROOT/src/sidecar"
        python3 << EOF
import json
with open("$AUTH_CONFIG") as f:
    config = json.load(f)

choice = "$auth_choice"
if choice == "1":
    config["require_auth"] = True
    print("✅ Authentication enabled")
elif choice == "2":
    config["require_auth"] = False
    print("⚠️  Authentication disabled - your system is now unprotected!")
elif choice == "3":
    config["require_auth_local"] = True
    print("✅ Local network authentication required")
elif choice == "4":
    config["require_auth_local"] = False
    print("✅ Local network authentication disabled")
else:
    print("❌ Invalid choice")
    exit(1)

with open("$AUTH_CONFIG", "w") as f:
    json.dump(config, f, indent=2)
EOF
        
        chmod 600 "$AUTH_CONFIG"
        echo ""
        echo -e "${YELLOW}🔄 Restart webbridge service: sudo systemctl restart webbridge${NC}"
        ;;
        
    4)
        echo -e "${BLUE}👋 Goodbye!${NC}"
        exit 0
        ;;
        
    *)
        echo -e "${RED}❌ Invalid choice${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}✅ Done!${NC}"
