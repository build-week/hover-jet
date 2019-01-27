# Usage

Run pymake on your repo, and it will generate a single CMakeLists.txt file containing all of the targets it can find. Here's the common case:

* pymake will create a library if `xyz.hh` and `xyz.cc` exist in the same folder
* pymake will create a *test* if `xyz_test.cc` exists and includes `gtest.hh`
* pymake will create a binary if it finds a `main()` function in a `.cc` file
* pymake will follow header includes to figure out dependencies. If it includes a header that corresponds to a library, it will link to that library
* If pymake doesn't support your a use case, just write a CMakeLists.txt in the sub-folder, and pymake won't create anything with that subfolder. As long as the headers in that subfolder have the same name as the libraries they belong to, `pymake` will correctly link to them.

```shell
python pymake.py --path my_repo/
```

Example folder structure
```shell
|-code_for_xyz
 |-xyz.hh # include "special_use_cases/special_code.hh", creating a dependency on library special_code
 |-xyz.cc # Forms a lib called xyz
 |-xyz_test.cc
|-special_use_cases
 |-CMakeLists.txt
 |-special_code.cc
 |-special_code.hh
```

If you somehow stumble upon this, decide to use it, and find that it isn't to your liking, shoot me an email and let's talk about it.

# Why

* Reducing the friction to doing simple things is of tremendous and often underestimated value
* The minor yak-shaves require at most minutes, meaning that the 8 hour exercise of implementing this tool is wasted, right?
    * Every yak-shave constitutes a loss of focus. Focus takes at least 30min to regain, effectively making any minor diversion at least a 30min diversion


# TODO
- More useful errors
- Hints when you haven't properly specified headers
- A "known libs" mapping from headers to lib objects
- Warn when library names are non-unique