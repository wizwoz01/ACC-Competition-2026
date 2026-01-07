# 🐳 Docker Setup - ACC 2026 Competition

This directory contains the Docker configuration for the ACC 2026 Self-Driving Car Competition development environment.

## Prerequisites

### System Requirements
- **OS:** Ubuntu 24.04 (recommended) or Windows with WSL2
- **GPU:** NVIDIA graphics card with CUDA support
- **RAM:** 16GB minimum, 32GB recommended
- **Storage:** 50GB free space minimum

### Required Software
- Docker Engine (20.10+)
- Docker Compose (v2.0+)
- NVIDIA Container Toolkit
- Git

## Quick Start

### 1. Install Docker

**Ubuntu:**
```bash
# Update packages
sudo apt update && sudo apt upgrade -y

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

**Windows (WSL2):**
- Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- Enable WSL2 backend in Docker settings

### 2. Install NVIDIA Container Toolkit

```bash
# Add NVIDIA GPG key
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

### 3. Clone Competition Resources

```bash
# Clone the ROS technical resources
git clone https://github.com/quanser/student-competition-resources-ros.git

# Navigate to the repository
cd student-competition-resources-ros
```

### 4. Build and Run Container

```bash
# Build the Docker image
docker compose build

# Start the container
docker compose up -d

# Enter the container
docker compose exec ros2_dev bash
```

## Directory Structure

```
Docker/
├── README.md           # This file
├── Dockerfile          # Custom Dockerfile 
├── docker-compose.yml  # Docker Compose configuration
└── .env               # Environment variables
```

## Useful Docker Commands

```bash
# List running containers
docker ps

# Stop containers
docker compose down

# View logs
docker compose logs -f

# Rebuild without cache
docker compose build --no-cache

# Clean up unused resources
docker system prune -a
```

## Troubleshooting

### GPU Not Detected
```bash
# Verify NVIDIA driver
nvidia-smi

# Test NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### Display Issues (for GUI apps)
```bash
# Allow X11 forwarding
xhost +local:docker
```

### Low QLabs Performance
- Ensure NVIDIA GPU is being used (not integrated graphics)
- Check NVIDIA Container Toolkit is properly installed
- See [QLabs Performance FAQ](https://github.com/quanser/student-competition-resources-ros/discussions)

## Resources

- [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros)
- [Docker Documentation](https://docs.docker.com/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

---

*Beach Autonomous Systems - CSULB*

