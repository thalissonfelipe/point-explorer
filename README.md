# point-explorer
A C++ application to explore some properties of points in Point Cloud images 

## Running
To run, you need Node 10. As it is a old version of node, I recomend you to use [nvm](https://github.com/nvm-sh/nvm).

Then, install `cmake-js` globally with:
```
npm install -g cmake-js
```

Finally, on the root folder of this project, run:
```
npm install
```

It will install the necessary dependencies and run `cmake-js compile` to build the node module from the C++ code. The build result can be found on `/build/Release/point_explorer.node`.
