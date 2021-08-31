# point-explorer

This project is a fork from https://github.com/MarcusVLMA/point-explorer and was developed during my undergraduate thesis.

A C++ application to explore some properties of points in Point Cloud images.

## Dependencies

To run this project you will need:

- [Point Cloud Library](https://pointclouds.org/)
- [CMake](https://cmake.org/) to build the output.

If you intend to generate the `.node` file, you will also need:

- Node 10 (I recommend using [nvm](https://github.com/nvm-sh/nvm) since this is a old node version).
- [CMake.js](https://github.com/cmake-js/cmake-js) to build using cmake.

## Building

This project can be compiled as follows:

### Regular C++ output

Once you have installed Point Cloud Library and CMake, just get into `src` and run:

```
cmake .
make
```

The executable output file can be found inside of `src/build` folder.

### To use with Node.js

You may want to use this algorithm with a web application, like I did in this [repository](https://github.com/thalisson/latin-pcd-viewer). A way to do this is generating the `.node` build, to use as a dependency in Node.js code.

As mencioned earlier, you will need Node.js 10. If you are using `nvm`, you can set Node version to 10 by running:

```
nvm install 10.23.0
nvm use 10
```

_Note that this will set your Node version only in the current terminal._

After that, install `cmake-js` globally with:

```
npm install -g cmake-js
```

Now, go into `nan-module` folder and run:

```
npm run install
npm run compile
```

This will install any necessary dependencies and compile the code with `cmake-js`.

The output file can be found inside of `nan-module/build` folder.

## Limitations

The `main` function inside `src/main.cpp` is used to extract data from geometric descriptors in the neighborhood of a given point (mainly the fiducial points) in a point cloud. This function is still very much in its primitive form, that is, it is always necessary to recompile the code in case of changes. Some of the main limitations will be presented here and all of them it is necessary to modify some lines of code:

- [ ] extract data using a given radius value or k number of neighbors.
- [ ] use other radius or k values.
- [ ] extract data from specific fiducial points (like nosetip, inner-left-eye...).
- [ ] landmarks clouds path and original clouds path are also hardcoded.
- [ ] provide the name of the file where the data will be saved.
