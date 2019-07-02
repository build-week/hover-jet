Hover-Jet
=========

# Look at the wiki for learning how to get started
Most of the commands in this file are copied from there.
https://github.com/build-week/hover-jet/wiki

# How to get the docker image
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

### Pulling the latest image

You will need to log out and back in for the `docker` group to be added before you can pull the image.

```shell
docker pull hoverjet/jet
```

# How to pull
### Pulling the first time
```
git clone https://github.com/build-week/hover-jet.git
cd hover-jet
git submodule init
git submodule update
```

```
git pull --rebase origin master
```

# How to Build

All compiling and running should be done *inside the docker*. There should be no exception to this.
It's typically that managing git and editing files is done outside of the docker, on the host machine

### How to build *nominally*
```shell
jet build <target>

jet run <bash command>
```

If this fails, you may have to do the submodule init steps.

### Using the shell inside the docker image (Sometimes useful for debugging)
Note: You should be using jet build or jet run, instead of these bash scripts
```shell
# Make sure the docker image is up to date
docker pull hoverjet/jet
# Enter the docker image
docker run -it -v ~/repos/hover-jet:/jet hoverjet/jet
```

Now, inside thedocker image
```shell
# Make sure you're in the docker image!!!!!!!
cd hover-jet

mkdir -p bin
cd bin
cmake ..
make -j7  # Where the 7 is the number of cores to use when building
```

### Using our Build-System

##### Using Internal Libraries

* If you *include* an internal library, the CMake dependencies and linking will automatically be determined
* Such includes must be specified from the root of the repo
    * `#include "infrastructure/comms/mqtt_subscriber.hh"` instead of `#include "mqtt_subscriber.hh"`
    * Pymake only inspects `""` includes, and ignores chevron includes, because chevrons are assumed to by system or external libs
* What if internal library isn't inspected by pymake? (Say it's been intentionally ignored, or has its own CMakeLists)
    * Then you must manually specify, in either the `.hh` or `.cc` of your library, `%deps(the_lib_i_want)`

##### Using External Libraries
* Third party libraries are typically not auto-linked through pymake, though this is an upcoming feature.
* Instead, we must manually specify dependencies
* Examples:
    * `%deps(opengl, glfw)`, `%deps(opencv)`


##### Defining Libraries

* A library *target* will be generated when a `.hh` and `.cc` file have the same name
    * For example: `my_lib.hh`, and `my_lib.cc`
    * The library will be called `my_lib`
    * All library names must be unique (This is, in principle, a CMake limitation)

* An implicit *header-only* library will be discovered whenever only a `.hh` exists
    * Things that include the header-only library will implicitly link against libraries used by the header-only lib

##### Defining Executables

* If your file contains a main function, an executable will be generated
* If your file contains "// %binary", an executable will be generated

##### Adding a third-party library

* Header-only libraries can be simply plopped into the third_party folder, and then
    * `include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib")` placed in the top-level CMakeLists.txt
        * Sometimes you have to be clever with the above command
* Libraries that require some compilation are tricky. In order of decreasing ease:
    * apt-package
    * compile from source and `make install`
    * include it in our repo, and munge its `CMake` chain to work with ours; This can often be very difficult

##### Debugging

* My thing isn't getting built!
    * Look at `/tmp/CMakeLists.txt`, and check that `add_library(my_lib)` or `add_executable(my_exec)` exists
    * Make sure that when `cmake ..` is run, there is no error generated. Sometimes it will be buried in a bit of text
    * Is there a `// %ignore` in your file (Check both the header and cc!)
    * Do `pymake -v info` in the `hover-jet` directory, and make sure your file is getting discovered

* My build takes a long time
    * The cmake cache gets invalidated often
    * Try `make -j3 <just_the_target_i_want>`, and rely on CI to asynchronously build *everything*

* There's a weird `target already exists` error, or something like it
    * This usually happens because your library name is not globally unique. Keep in mind, CMake effectively requires all library names to be globally unique.
    * If this is a big problem, we can use pymake to auto-generate unique library names

* There's a `header XXXXXX.hh unknown in YYYYYY.cc`
    * Usually, this means `YYYYYY.cc` is including a header that isn't real
    * If it's an external library that isn't in our repository, use `<>` includes, then pymake won't require its existence
    * Make sure the `#include` path is from the root of the repo, and not relative
        * `#include "infrastructure/comms/mqtt_subscriber.hh"` instead of `#include "mqtt_subscriber.hh"`

* No definition for XXXX error, or could not find library `-lmy_library`, or `failed to link`
    * This is *usually* because `%deps()` is being used for a library name that doesn't exist
        * If it's external, check `ldconfig -p | grep <lib_name_I_expect_to_exist>`
            * If ldconfig has the lib, but you still see the error, you must add `find_package(lib-name)` in the top-level CMake
        * If it's internal, try `git grep -rni "add_library(lib_name_I_expect_to_exist"` (With the closing parenthesis excluded)
        * If these are not the case, go find Jake, this is a bug

* third_party/experiments does not contain a CMakeLists.txt file
    * do `git submodule init; git submodule update`
