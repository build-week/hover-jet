Hover-Jet
=========

To get started, take a look at the [wiki](https://github.com/build-week/hover-jet/wiki).
Most of the Bash snippets in this file are copied from there.


# How to pull

### Pulling the first time

This step assumes that you don't have a Git Hub account, but feel free to use
SSH to pull the repository, as is described in the wiki, if you do.

```shell
# Clone the hover-jet repository
git clone https://github.com/build-week/hover-jet.git
cd hover-jet

# Initialize the experiments submodule
git submodule init third_party/experiments/
git submodule update third_party/experiments/
```

### Pulling in new changes from upstream

```shell
# Rebase any local changes onto upstream's master branch
git pull --rebase origin master
```


# How to bootstrap your shell environment

### Using the repository's bootstrap script

```shell
# Run the bootstrap script to modify the "$PATH" this shell and all new shells
./infrastructure/scripts/environment_bootstrap.sh
```


# How to get the Docker image

### Installing Docker Community Edition

```shell
# Install prerequisites to add the Docker CE PPA
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl software-properties-common

# Add the Docker CE PPA's GPG key using GPG as a middleman
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --import
gpg --export 0EBFCD88 | sudo apt-key add

# Add the Docker CE PPA's APT sources list
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# Install the Docker CE package
sudo apt-get update
sudo apt-get install docker-ce

# Add the user to the "docker" group so they can orchestrate the daemon
sudo usermod -aG docker "$USER"
```

### Pulling the latest jet Docker image

You will need to log out and back in for the `docker` group to be added before you can pull the image.

```shell
# Pull the latest jet Docker image
jet image --pull
```


# How to Build

All compiling and running should be done *inside the Docker container*. There should be no exception to this.
It's typically that managing git and editing files is done outside of the Docker container, on the host machine

### How to build *nominally*

```shell
# Build the entire project
jet build

# Execute a shell in the jet Docker container
jet run
```

If this fails, you may have missed the submodule init steps from the beginning.
