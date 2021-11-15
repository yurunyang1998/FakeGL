//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5812M Foundations of Modelling & Rendering
//  User Interface for Coursework
//
//  September, 2020
//
//  ------------------------
//  FakeGL.cpp
//  ------------------------
//  
//  A unit for implementing OpenGL workalike calls
//  
///////////////////////////////////////////////////

#include "FakeGL.h"
#include <math.h>

//-------------------------------------------------//
//                                                 //
// CONSTRUCTOR / DESTRUCTOR                        //
//                                                 //
//-------------------------------------------------//

// constructor
FakeGL::FakeGL()
    { // constructor
        std::cout<<"construct FakeGL"<<std::endl;


    } // constructor

// destructor
FakeGL::~FakeGL()
    { // destructor
    } // destructor

//-------------------------------------------------//
//                                                 //
// GEOMETRIC PRIMITIVE ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// starts a sequence of geometric primitives
void FakeGL::Begin(unsigned int PrimitiveType)
    { // Begin()

       this->currentPrimitive = PrimitiveType;

    } // Begin()

// ends a sequence of geometric primitives
void FakeGL::End()
    { // End()
        this->currentPrimitive = -1;
    } // End()

// sets the size of a point for drawing
void FakeGL::PointSize(float size)
    { // PointSize()
        this->pointSize = size;
    } // PointSize()

// sets the width of a line for drawing purposes
void FakeGL::LineWidth(float width)
    { // LineWidth()
        this->lineWidth = width;
    } // LineWidth()

//-------------------------------------------------//
//                                                 //
// MATRIX MANIPULATION ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// set the matrix mode (i.e. which one we change)   
void FakeGL::MatrixMode(unsigned int whichMatrix)
    { // MatrixMode()
    } // MatrixMode()

// pushes a matrix on the stack
void FakeGL::PushMatrix()
    { // PushMatrix()
    } // PushMatrix()

// pops a matrix off the stack
void FakeGL::PopMatrix()
    { // PopMatrix()
    } // PopMatrix()

// load the identity matrix
void FakeGL::LoadIdentity()
    { // LoadIdentity()
    } // LoadIdentity()

// multiply by a known matrix in column-major format
void FakeGL::MultMatrixf(const float *columnMajorCoordinates)
    { // MultMatrixf()
    } // MultMatrixf()

// sets up a perspective projection matrix
void FakeGL::Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Frustum()
    } // Frustum()

// sets an orthographic projection matrix
void FakeGL::Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Ortho()
    } // Ortho()

// rotate the matrix
void FakeGL::Rotatef(float angle, float axisX, float axisY, float axisZ)
    { // Rotatef()
    } // Rotatef()

// scale the matrix
void FakeGL::Scalef(float xScale, float yScale, float zScale)
    { // Scalef()
    } // Scalef()

// translate the matrix
void FakeGL::Translatef(float xTranslate, float yTranslate, float zTranslate)
    { // Translatef()
    } // Translatef()

// sets the viewport
void FakeGL::Viewport(int x, int y, int width, int height)
    { // Viewport()
    } // Viewport()

//-------------------------------------------------//
//                                                 //
// VERTEX ATTRIBUTE ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// sets colour with floating point
void FakeGL::Color3f(float red, float green, float blue)
    { // Color3f()
    } // Color3f()

// sets material properties
void FakeGL::Materialf(unsigned int parameterName, const float parameterValue)
    { // Materialf()
    } // Materialf()

void FakeGL::Materialfv(unsigned int parameterName, const float *parameterValues)
    { // Materialfv()
    } // Materialfv()

// sets the normal vector
void FakeGL::Normal3f(float x, float y, float z)
    { // Normal3f()
    } // Normal3f()

// sets the texture coordinates
void FakeGL::TexCoord2f(float u, float v)
    { // TexCoord2f()
    } // TexCoord2f()

// sets the vertex & launches it down the pipeline
void FakeGL::Vertex3f(float x, float y, float z)
    { // Vertex3f()
        vertexWithAttributes vertex(x,y,z);
        this->vertexQueue.push_back(vertex);
        TransformVertex();
    } // Vertex3f()

//-------------------------------------------------//
//                                                 //
// STATE VARIABLE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// disables a specific flag in the library
void FakeGL::Disable(unsigned int property)
    { // Disable()
    } // Disable()

// enables a specific flag in the library
void FakeGL::Enable(unsigned int property)
    { // Enable()
    } // Enable()

//-------------------------------------------------//
//                                                 //
// LIGHTING STATE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// sets properties for the one and only light
void FakeGL::Light(int parameterName, const float *parameterValues)
    { // Light()
    } // Light()

//-------------------------------------------------//
//                                                 //
// TEXTURE PROCESSING ROUTINES                     //
//                                                 //
// Note that we only allow one texture             //
// so glGenTexture & glBindTexture aren't needed   //
//                                                 //
//-------------------------------------------------//

// sets whether textures replace or modulate
void FakeGL::TexEnvMode(unsigned int textureMode)
    { // TexEnvMode()
    } // TexEnvMode()

// sets the texture image that corresponds to a given ID
void FakeGL::TexImage2D(const RGBAImage &textureImage)
    { // TexImage2D()
    } // TexImage2D()

//-------------------------------------------------//
//                                                 //
// FRAME BUFFER ROUTINES                           //
//                                                 //
//-------------------------------------------------//

// clears the frame buffer
void FakeGL::Clear(unsigned int mask)
    { // Clear()
        int width = this->frameBuffer.width;
        int height = this->frameBuffer.height;
        for(int i=0;i<height;i++){
            for(int j=0;j<width;j++){

                this->frameBuffer[i][j].red=this->backGroundColor.red;
                this->frameBuffer[i][j].blue=this->backGroundColor.blue;
                this->frameBuffer[i][j].green=this->backGroundColor.green;
                this->frameBuffer[i][j].alpha=this->backGroundColor.alpha;
            }
        }


} // Clear()

// sets the clear colour for the frame buffer
void FakeGL::ClearColor(float red, float green, float blue, float alpha)
    { // ClearColor()
           this->backGroundColor=RGBAValue(red*255,green*255,blue*255, alpha*255);
    } // ClearColor()

//-------------------------------------------------//
//                                                 //
// MAJOR PROCESSING ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// transform one vertex & shift to the raster queue
void FakeGL::TransformVertex()
    { // TransformVertex()
        auto vertex = this->vertexQueue.front();
        vertexQueue.pop_front();

        // implement matrix


        screenVertexWithAttributes  screenVertex(vertex.position.x , vertex.position.y, vertex.position.z);
        this->rasterQueue.push_back(screenVertex);

        RasterisePrimitive();

    } // TransformVertex()

// rasterise a single primitive if there are enough vertices on the queue
bool FakeGL::RasterisePrimitive()
    { // RasterisePrimitive()
    switch (this->currentPrimitive) {
        case 0:{ //rasterise a vertex
            if(this->rasterQueue.size()>=1){
                std::cout<<"rasterise point"<<std::endl;
                auto vertex = rasterQueue.front();
                rasterQueue.pop_front();
                //screenVertexWithAttributes rasterVertex(vertex.position.x, vertex.position.y, vertex.position.z);
                RasterisePoint(vertex);
                return true;
            }else
                return false;
        }
        case 1:{  //rasterise a line
            if(this->rasterQueue.size()>=2){

                std::cout<<"rasterise line"<<std::endl;
                auto vertex1 = rasterQueue.front();rasterQueue.pop_front();
                auto vertex2 = rasterQueue.front();rasterQueue.pop_front();
//                screenVertexWithAttributes rasterVertex1(vertex1.position.x, vertex1.position.y, vertex1.position.z);
//                screenVertexWithAttributes rasterVertex2(vertex2.position.x, vertex2.position.y, vertex2.position.z);

                this->RasteriseLineSegment(vertex1, vertex2);
                return true;
            }else
                return false;
        }
        case 2:{     // rasterise a triangle

            std::cout<<"render triangle"<<std::endl;

            if(this->rasterQueue.size()>=3){
                auto vertex1 = rasterQueue.front();rasterQueue.pop_front();
                auto vertex2 = rasterQueue.front();rasterQueue.pop_front();
                auto vertex3 = rasterQueue.front();rasterQueue.pop_front();
                screenVertexWithAttributes rasterVertex1(vertex1.position.x*100, vertex1.position.y*100, vertex1.position.z*100);
                screenVertexWithAttributes rasterVertex2(vertex2.position.x*100, vertex2.position.y*100, vertex2.position.z*100);
                screenVertexWithAttributes rasterVertex3(vertex3.position.x*100, vertex3.position.y*100, vertex3.position.z*100);


                RasteriseTriangle(rasterVertex1, rasterVertex2, rasterVertex3);
                return true;
            }else
                return false;
        }
      }
    return false;   //can't rasterise a primitive;
    } // RasterisePrimitive()

// rasterises a single point
void FakeGL::RasterisePoint(screenVertexWithAttributes &vertex0)
    { // RasterisePoint()
        std::cout<<"rasterise"<<std::endl;
        int x = vertex0.position.x;
        int y = vertex0.position.y;
        int z = vertex0.position.z;
        int pointSize = this->pointSize;
        if(pointSize==0){
            std::cout<<"pointsize is zero"<<std::endl;
            return;
        }

        for(int _x=x-(pointSize/2);_x<=x+(pointSize/2);_x++){
            for(int _y=y-(pointSize/2);_y<=y+(pointSize/2);_y++){

                //TODO: implement depth test

                fragmentWithAttributes shaderVertex(_x,_y,vertex0.colour);
                this->fragmentQueue.push_back(shaderVertex);

            }
        }

        ProcessFragment();






    } // RasterisePoint()

// rasterises a single line segment
void FakeGL::RasteriseLineSegment(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1)
    { // RasteriseLineSegment()

       int x0 = vertex0.position.x, y0 = vertex0.position.y;
       int x1 = vertex1.position.x, y1 = vertex1.position.y;

       std::cout<<x0<<" "<<y0<<std::endl;
       std::cout<<x1<<" "<<y1<<std::endl;
       bool steep = false;
           if (std::abs(x0-x1)<std::abs(y0-y1)) {
               std::swap(x0, y0);
               std::swap(x1, y1);
               steep = true;
           }
           if (x0>x1) {
               std::swap(x0, x1);
               std::swap(y0, y1);
           }
           int dx = x1-x0;
           int dy = y1-y0;
           int derror2 = std::abs(dy)*2;
           int error2 = 0;
           int y = y0;
           for (int x=x0; x<=x1; x++) {
               if (steep) {
                   fragmentWithAttributes fragmentVertex(x, y, vertex0.colour);
                   this->fragmentQueue.push_back(fragmentVertex);
               } else {
                   fragmentWithAttributes fragmentVertex(y, x, vertex0.colour);
                   this->fragmentQueue.push_back(fragmentVertex);
               }
               error2 += derror2;
               if (error2 > dx) {
                   y += (y1>y0?1:-1);
                   error2 -= dx*2;
               }
           }




            ProcessFragment();


    } // RasteriseLineSegment()

// rasterises a single triangle
void FakeGL::RasteriseTriangle(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1, screenVertexWithAttributes &vertex2)
    { // RasteriseTriangle()
    // compute a bounding box that starts inverted to frame size
    // clipping will happen in the raster loop proper
    float minX = frameBuffer.width, maxX = 0.0;
    float minY = frameBuffer.height, maxY = 0.0;
    
    // test against all vertices
    if (vertex0.position.x < minX) minX = vertex0.position.x;
    if (vertex0.position.x > maxX) maxX = vertex0.position.x;
    if (vertex0.position.y < minY) minY = vertex0.position.y;
    if (vertex0.position.y > maxY) maxY = vertex0.position.y;
    
    if (vertex1.position.x < minX) minX = vertex1.position.x;
    if (vertex1.position.x > maxX) maxX = vertex1.position.x;
    if (vertex1.position.y < minY) minY = vertex1.position.y;
    if (vertex1.position.y > maxY) maxY = vertex1.position.y;
    
    if (vertex2.position.x < minX) minX = vertex2.position.x;
    if (vertex2.position.x > maxX) maxX = vertex2.position.x;
    if (vertex2.position.y < minY) minY = vertex2.position.y;
    if (vertex2.position.y > maxY) maxY = vertex2.position.y;

    // now for each side of the triangle, compute the line vectors
    Cartesian3 vector01 = vertex1.position - vertex0.position;
    Cartesian3 vector12 = vertex2.position - vertex1.position;
    Cartesian3 vector20 = vertex0.position - vertex2.position;

    // now compute the line normal vectors
    Cartesian3 normal01(-vector01.y, vector01.x, 0.0);  
    Cartesian3 normal12(-vector12.y, vector12.x, 0.0);  
    Cartesian3 normal20(-vector20.y, vector20.x, 0.0);  

    // we don't need to normalise them, because the square roots will cancel out in the barycentric coordinates
    float lineConstant01 = normal01.dot(vertex0.position);
    float lineConstant12 = normal12.dot(vertex1.position);
    float lineConstant20 = normal20.dot(vertex2.position);

    // and compute the distance of each vertex from the opposing side
    float distance0 = normal12.dot(vertex0.position) - lineConstant12;
    float distance1 = normal20.dot(vertex1.position) - lineConstant20;
    float distance2 = normal01.dot(vertex2.position) - lineConstant01;

    // if any of these are zero, we will have a divide by zero error
    // but notice that if they are zero, the vertices are collinear in projection and the triangle is edge on
    // we can render that as a line, but the better solution is to render nothing.  In a surface, the adjacent
    // triangles will eventually take care of it
    if ((distance0 == 0) || (distance1 == 0) || (distance2 == 0))
        return; 

    // create a fragment for reuse
    fragmentWithAttributes rasterFragment;

    // loop through the pixels in the bounding box
    for (rasterFragment.row = minY; rasterFragment.row <= maxY; rasterFragment.row++)
        { // per row
        // this is here so that clipping works correctly
        if (rasterFragment.row < 0) continue;
        if (rasterFragment.row >= frameBuffer.height) continue;
        for (rasterFragment.col = minX; rasterFragment.col <= maxX; rasterFragment.col++)
            { // per pixel
            // this is also for correct clipping
            if (rasterFragment.col < 0) continue;
            if (rasterFragment.col >= frameBuffer.width) continue;
            
            // the pixel in cartesian format
            Cartesian3 pixel(rasterFragment.col, rasterFragment.row, 0.0);
            
            // right - we have a pixel inside the frame buffer AND the bounding box
            // note we *COULD* compute gamma = 1.0 - alpha - beta instead
            float alpha = (normal12.dot(pixel) - lineConstant12) / distance0;           
            float beta = (normal20.dot(pixel) - lineConstant20) / distance1;            
            float gamma = (normal01.dot(pixel) - lineConstant01) / distance2;           

            // now perform the half-plane test
            if ((alpha < 0.0) || (beta < 0.0) || (gamma < 0.0))
                continue;

            // compute colour
            rasterFragment.colour = alpha * vertex0.colour + beta * vertex1.colour + gamma * vertex2.colour; 

            // now we add it to the queue for fragment processing
            fragmentQueue.push_back(rasterFragment);
            std::cout<<rasterFragment.row<<" "<<rasterFragment.col<<std::endl;
            } // per pixel
        } // per row
    ProcessFragment();
    } // RasteriseTriangle()

// process a single fragment
void FakeGL::ProcessFragment()
    { // ProcessFragment()
        while(!this->fragmentQueue.empty()){
            auto fragment = this->fragmentQueue.front();
            this->fragmentQueue.pop_front();

            int x = fragment.col;
            int y = fragment.row;
            this->frameBuffer[x][y] = fragment.colour;

        }



    } // ProcessFragment()

// standard routine for dumping the entire FakeGL context (except for texture / image)
std::ostream &operator << (std::ostream &outStream, FakeGL &fakeGL)
    { // operator <<
    outStream << "=========================" << std::endl;
    outStream << "Dumping FakeGL Context   " << std::endl;
    outStream << "=========================" << std::endl;


    outStream << "-------------------------" << std::endl;
    outStream << "Vertex Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.vertexQueue.begin(); vertex < fakeGL.vertexQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.vertexQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Raster Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.rasterQueue.begin(); vertex < fakeGL.rasterQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.rasterQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Fragment Queue:          " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto fragment = fakeGL.fragmentQueue.begin(); fragment < fakeGL.fragmentQueue.end(); fragment++)
        { // per matrix
        outStream << "Fragment " << fragment - fakeGL.fragmentQueue.begin() << std::endl;
        outStream << *fragment;
        } // per matrix


    return outStream;
    } // operator <<

// subroutines for other classes
std::ostream &operator << (std::ostream &outStream, vertexWithAttributes &vertex)
    { // operator <<
    std::cout << "Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

	// you

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, screenVertexWithAttributes &vertex) 
    { // operator <<
    std::cout << "Screen Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, fragmentWithAttributes &fragment)
    { // operator <<
    std::cout << "Fragment With Attributes" << std::endl;
    std::cout << "Row:        " << fragment.row << std::endl;
    std::cout << "Col:        " << fragment.col << std::endl;
    std::cout << "Colour:     " << fragment.colour << std::endl;

    return outStream;
    } // operator <<


    
    
