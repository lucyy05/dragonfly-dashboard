#!/bin/bash

# Configuration
INTERFACE="wlp62s0"  # Your network interface
ENV_FILE="frontend/.env"  # Path to your .env file
BACKEND_PORT="8000"  # Your backend port

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}🔍 Detecting IP address for interface: $INTERFACE${NC}"

# Get the current IP address
CURRENT_IP=$(ip addr show $INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n1)

if [ -z "$CURRENT_IP" ]; then
    echo -e "${RED}❌ Error: Could not detect IP address for interface $INTERFACE${NC}"
    echo "Available interfaces:"
    ip addr show | grep -E '^[0-9]+:' | awk '{print $2}' | sed 's/://'
    exit 1
fi

echo -e "${GREEN}✅ Current IP detected: $CURRENT_IP${NC}"

# Create .env file if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    echo -e "${YELLOW}📝 Creating new .env file at $ENV_FILE${NC}"
    touch "$ENV_FILE"
fi

# Check if IP has changed
if [ -f "$ENV_FILE" ] && grep -q "REACT_APP_API_URL=http://$CURRENT_IP:$BACKEND_PORT" "$ENV_FILE"; then
    echo -e "${GREEN}✅ IP address unchanged. No update needed.${NC}"
    exit 0
fi

# Backup existing .env file
if [ -f "$ENV_FILE" ] && [ -s "$ENV_FILE" ]; then
    cp "$ENV_FILE" "$ENV_FILE.backup.$(date +%Y%m%d_%H%M%S)"
    echo -e "${YELLOW}📋 Backup created: $ENV_FILE.backup.$(date +%Y%m%d_%H%M%S)${NC}"
fi

# Update or create .env file
echo -e "${YELLOW}🔄 Updating .env file...${NC}"

# Remove existing API_URL and WS_URL lines
sed -i '/^REACT_APP_API_URL=/d' "$ENV_FILE" 2>/dev/null
sed -i '/^REACT_APP_WS_URL=/d' "$ENV_FILE" 2>/dev/null

# Add new lines
echo "REACT_APP_API_URL=http://$CURRENT_IP:$BACKEND_PORT" >> "$ENV_FILE"
echo "REACT_APP_WS_URL=ws://$CURRENT_IP:$BACKEND_PORT/ws/robot-data" >> "$ENV_FILE"

echo -e "${GREEN}✅ .env file updated successfully!${NC}"
echo -e "${GREEN}📝 New configuration:${NC}"
echo -e "   API URL: http://$CURRENT_IP:$BACKEND_PORT"
echo -e "   WebSocket URL: ws://$CURRENT_IP:$BACKEND_PORT/ws/robot-data"
echo ""
echo -e "${YELLOW}⚠️  Remember to restart your React development server for changes to take effect!${NC}"

# Optional: Show the updated .env file contents
echo -e "${YELLOW}📄 Current .env file contents:${NC}"
cat "$ENV_FILE"
