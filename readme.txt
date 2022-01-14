My method describe:

The method that I adopted is Loop subdivision,  is divide vertexs to new vertexs and old vertexs.
First , I updata these old vertexs ' position and save it.
Then, I using some halfedge data structure to save those halfEdge's relationship, which save each's halfedge's pair, next, vertexs, etc.
In each subdivision time, I generate some new vertexs by old vertexs and split each triangle face to 4 new smaller faces.
and re-calculate new normal data for each new face.


To compile on feng-linux / feng-gps:

module add qt/5.13.0
qmake -project QT+=opengl
qmake
make
--------------------------------------

To compile on OSX:
Use Homebrew to install qt

qmake -project QT+=opengl
qmake
make
--------------------------------------

To compile on Windows:
Install QT 5.13.0 
Update graphics card driver
Double click FakeGLRenderWindow_win.pro to open in QTCreator 
Select the platform to compile to (32 or 64 bits)
Click details to select the build folder
Click Configure Project
--------------------------------------

To run on feng-linux / feng-gps:
./FakeGLRenderWindowRelease ../path_to/model.obj ../path_to/texture.ppm

---------------------------------------------

To run on OSX:
./FakeGLRenderWindowRelease.app/Contents/MacOS/FakeGLRenderWindowRelease  ../path_to/model.obj ../path_to/texture.ppm

---------------------------------------------
To run on Windows
./FakeGLRenderWindowRelease.exe ../path_to/model.obj ../path_to/texture.ppm
or
Click projects
Select "Run" on the left side menu under the active configuration
Add "../path_to/model.obj ../path_to/texture.ppm" to command line arguments
Click Run/Debug on the bottom left side.