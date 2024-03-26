# F1tenth QuickStart
# 1. Flash Jetpack

# 2. Setup SSH and Static IP

# 3. Install essential libraries
    sudo apt update && sudo apt install -y python3-dev python3-pip python3-setuptools python3-venv build-essential git curl

# 4. Install Jtop
    sudo pip3 install -U jetson-stats
    sudo reboot now

# 5. Setup Nvidia Container Runtime

# 6. Setup Docker for use without root
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker

# 7. Setup Docker to use the buildx plugin
    sudo apt update
    sudo apt install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg 
		echo \
		"deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
		"$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
		sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt update
    sudo apt install -y docker-buildx-plugin
    docker buildx install

# 8. 