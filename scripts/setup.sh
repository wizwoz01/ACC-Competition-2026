#!/bin/bash
# Beach Autonomous Systems - Project Setup Script
# ACC 2026 Self-Driving Car Competition

echo "========================================"
echo "Beach Autonomous Systems - Setup Script"
echo "ACC 2026 Self-Driving Car Competition"
echo "========================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running in correct directory
if [ ! -f "README.md" ]; then
    echo -e "${RED}Error: Please run this script from the project root directory${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 1: Checking dependencies...${NC}"

# Check for Git
if command -v git &> /dev/null; then
    echo -e "${GREEN}✓ Git installed${NC}"
else
    echo -e "${RED}✗ Git not found. Please install Git.${NC}"
    exit 1
fi

# Check for Docker
if command -v docker &> /dev/null; then
    echo -e "${GREEN}✓ Docker installed${NC}"
else
    echo -e "${RED}✗ Docker not found. Please install Docker.${NC}"
    exit 1
fi

# Check for Docker Compose
if command -v docker-compose &> /dev/null || docker compose version &> /dev/null; then
    echo -e "${GREEN}✓ Docker Compose installed${NC}"
else
    echo -e "${RED}✗ Docker Compose not found. Please install Docker Compose.${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 2: Cloning ROS Technical Resources...${NC}"

if [ -d "student-competition-resources-ros" ]; then
    echo -e "${GREEN}✓ ROS resources already cloned${NC}"
    cd student-competition-resources-ros
    git pull
    cd ..
else
    git clone https://github.com/quanser/student-competition-resources-ros.git
    echo -e "${GREEN}✓ ROS resources cloned${NC}"
fi

echo -e "${YELLOW}Step 3: Creating required directories...${NC}"

# Create directories if they don't exist
mkdir -p src/perception
mkdir -p src/planning
mkdir -p src/control
mkdir -p src/localization
mkdir -p src/bringup
mkdir -p docs/progress
mkdir -p docs/meetings
mkdir -p logs

echo -e "${GREEN}✓ Directories created${NC}"

echo ""
echo "========================================"
echo -e "${GREEN}Setup Complete!${NC}"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. cd student-competition-resources-ros"
echo "2. Follow Docker setup instructions"
echo "3. Start developing in src/ directory"
echo ""
echo "Good luck, Beach Autonomous Systems! 🏖️"

