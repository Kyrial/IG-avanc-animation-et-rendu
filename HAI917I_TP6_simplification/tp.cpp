// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <map>

#include <algorithm>
#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"


struct Triangle {
    inline Triangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;   v[1] = v1;   v[2] = v2;
    }
    unsigned int & operator [] (unsigned int iv) { return v[iv]; }
    unsigned int operator [] (unsigned int iv) const { return v[iv]; }
    inline virtual ~Triangle () {}
    inline Triangle & operator = (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
        return (*this);
    }
    // membres :
    unsigned int v[3];
};


struct Mesh {
    std::vector< Vec3 > vertices; //List of mesh vertices positions
    std::vector< Vec3 > normals;
    std::vector< Triangle > triangles;
    std::vector< Vec3 > triangle_normals;

    void computeTrianglesNormals(){

        triangle_normals.clear();
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            const Vec3 & e0 = vertices[triangles[i][1]] - vertices[triangles[i][0]];
            const Vec3 & e1 = vertices[triangles[i][2]] - vertices[triangles[i][0]];
            Vec3 n = Vec3::cross( e0, e1 );
            n.normalize();
            triangle_normals.push_back( n );
        }
    }

    void computeVerticesNormals(){

        normals.clear();
        normals.resize( vertices.size(), Vec3(0., 0., 0.) );
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            for( unsigned int t = 0 ; t < 3 ; t++ )
                normals[ triangles[i][t] ] += triangle_normals[i];
        }
        for( unsigned int i = 0 ; i < vertices.size() ;i++ )
            normals[ i ].normalize();

    }

    void computeNormals(){
        computeTrianglesNormals();
        computeVerticesNormals();
    }
};
Mesh mesh;

struct Voxel{
int cube[8];
int id=-1;
std::vector< Vec3 > pointInCellule;
};


struct cellule{
    std::vector< Vec3 > pointInCellule;
};

struct Grid{
    std::vector< Vec3 > quadrillage;
    std::vector< cellule > cellules;
    std::vector< Voxel > voxels; 
    int x=60;
    int y=60;
    int z=60;
    std::vector<std::vector<std::vector< Vec3 >>> quadrillage3d;// (x, std::vector<std::vector<Vec3>> (y, std::vector<Vec3>(z,Vec3(0,0,0))));


    Vec3 BBmin=Vec3(-1,-1,-1);    
    Vec3 BBmax=Vec3(1,1,1);
    std::vector< Vec3 > Sommet;

    //std::map<int,std::vector<Vec3>> dicoIndice;

}grid;



bool display_normals;
bool display_smooth_normals;
bool display_mesh;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 1600;
static unsigned int SCREENHEIGHT = 900;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static bool fullScreen = false;


// ------------------------------------


void initBB(Grid & grid,std::vector<Vec3>const & positions  ){
    grid.BBmin=positions[0];
    grid.BBmax=positions[0];
 
    for(int i = 0; i< positions.size();i++){
        if(positions[i][0]< grid.BBmin[0])
            grid.BBmin[0]= positions[i][0];
        if(positions[i][1]< grid.BBmin[1])
            grid.BBmin[1]= positions[i][1];
        if(positions[i][2]< grid.BBmin[2])
            grid.BBmin[2]= positions[i][2];         

        if(positions[i][0]> grid.BBmax[0])
            grid.BBmax[0]= positions[i][0];
        if(positions[i][1]> grid.BBmax[1])
            grid.BBmax[1]= positions[i][1];
        if(positions[i][2]> grid.BBmax[2])
            grid.BBmax[2]= positions[i][2]; 

    }
grid.BBmin=grid.BBmin- (grid.BBmax-grid.BBmin)/10;
    grid.BBmax=grid.BBmax+ (grid.BBmax-grid.BBmin)/10;
}
void initVoxel(Grid & grid){

    for(int i=0;i<grid.x-1;i++)
        for(int j=0;j<grid.y-1;j++)
            for(int k=0;k<grid.z-1;k++){
				int count = i*grid.y*grid.z + j* grid.z + k;
                Voxel voxels;
				//printf("cube %i, %i, %i, %i, %i, %i, %i, %i \n", 
            voxels.cube[0]=count;
            voxels.cube[1]=count+1;
            voxels.cube[2]=count+grid.z;
            voxels.cube[3]=count+grid.z+1;
            voxels.cube[4]=count+grid.z*grid.y;
            voxels.cube[5]=count+grid.z*grid.y+1;
            voxels.cube[6]=count+grid.z*grid.y+grid.z;
            voxels.cube[7]=count+grid.z*grid.y+grid.z+1;
            grid.voxels.push_back(voxels);   
            }
}

void initQuadrillage(Grid & grid ){

    float Xmin = grid.BBmin[0];
    float Ymin = grid.BBmin[1];
    float Zmin = grid.BBmin[2];

    float Xmax = grid.BBmax[0];
    float Ymax = grid.BBmax[1];
    float Zmax = grid.BBmax[2];


   float intervalX=(Xmax-Xmin)/(float)(grid.x-1);
   float intervalY=(Ymax-Ymin)/(float)(grid.y-1);
   float intervalZ=(Zmax-Zmin)/(float)(grid.z-1);
   
    grid.quadrillage.resize(grid.x*grid.y*grid.z);
    grid.cellules.resize(grid.x*grid.y*grid.z);
    for(int i=0;i<grid.x;i++){
        for(int j=0;j<grid.y;j++){
            for(int k=0;k<grid.z;k++){
                grid.quadrillage[i*grid.y*grid.z + j* grid.z + k] = Vec3(Xmin+intervalX*i, Ymin+intervalY*j, Zmin+intervalZ*k);
                }
        }
    }    
}













// ------------------------------------
// File I/O
// ------------------------------------
bool saveOFF( const std::string & filename ,
              std::vector< Vec3 > & i_vertices ,
              std::vector< Vec3 > & i_normals ,
              std::vector< Triangle > & i_triangles,
              bool save_normals = true ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl ;

    unsigned int n_vertices = i_vertices.size() , n_triangles = i_triangles.size();
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << i_vertices[v][0] << " " << i_vertices[v][1] << " " << i_vertices[v][2] << " ";
        if (save_normals) myfile << i_normals[v][0] << " " << i_normals[v][1] << " " << i_normals[v][2] << std::endl;
        else myfile << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << i_triangles[f][0] << " " << i_triangles[f][1] << " " << i_triangles[f][2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

void openOFF( std::string const & filename,
              std::vector<Vec3> & o_vertices,
              std::vector<Vec3> & o_normals,
              std::vector< Triangle > & o_triangles )
{
    std::ifstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open())
    {
        std::cout << filename << " cannot be opened" << std::endl;
        return;
    }

    std::string magic_s;

    myfile >> magic_s;

    if( magic_s != "OFF" )
    {
        std::cout << magic_s << " != OFF :   We handle ONLY *.off files." << std::endl;
        myfile.close();
        exit(1);
    }

    int n_vertices , n_faces , dummy_int;
    myfile >> n_vertices >> n_faces >> dummy_int;

    o_vertices.clear();

    for( int v = 0 ; v < n_vertices ; ++v )
    {
        float x , y , z ;

        myfile >> x >> y >> z ;
        o_vertices.push_back( Vec3( x , y , z ) );

    }

    o_triangles.clear();
    for( int f = 0 ; f < n_faces ; ++f )
    {
        int n_vertices_on_face;
        myfile >> n_vertices_on_face;

        if( n_vertices_on_face == 3 )
        {
            unsigned int _v1 , _v2 , _v3;
            myfile >> _v1 >> _v2 >> _v3;

            o_triangles.push_back(Triangle( _v1, _v2, _v3 ));
        }
        else if( n_vertices_on_face == 4 )
        {
            unsigned int _v1 , _v2 , _v3 , _v4;
            myfile >> _v1 >> _v2 >> _v3 >> _v4;

            o_triangles.push_back(Triangle(_v1, _v2, _v3 ));
            o_triangles.push_back(Triangle(_v1, _v3, _v4));
        }
        else
        {
            std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face" << std::endl;
            myfile.close();
            exit(1);
        }
    }

}

// ------------------------------------

void computeBBox( std::vector< Vec3 > const & i_positions, Vec3 & bboxMin, Vec3 & bboxMax ) {
    bboxMin = Vec3 ( FLT_MAX , FLT_MAX , FLT_MAX );
    bboxMax = Vec3 ( FLT_MIN , FLT_MIN , FLT_MIN );
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        for( unsigned int coord = 0 ; coord < 3 ; ++coord ) {
            bboxMin[coord] = std::min<float>( bboxMin[coord] , i_positions[pIt][coord] );
            bboxMax[coord] = std::max<float>( bboxMax[coord] , i_positions[pIt][coord] );
        }
    }
}

// ------------------------------------
// Application initialization
// ------------------------------------
void initLight () {
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f,-16.0f,-50.0f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};

    glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv (GL_LIGHT1, GL_SPECULAR, color1);
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);
}

void init () {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    initLight ();
    glCullFace (GL_BACK);
    glDisable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    display_normals = false;
    display_mesh = true;
    display_smooth_normals = true;

}


// ------------------------------------
// Rendering.
// ------------------------------------

void drawVector( Vec3 const & i_from, Vec3 const & i_to ) {

    glBegin(GL_LINES);
    glVertex3f( i_from[0] , i_from[1] , i_from[2] );
    glVertex3f( i_to[0] , i_to[1] , i_to[2] );
    glEnd();
}

void drawAxis( Vec3 const & i_origin, Vec3 const & i_direction ) {

    glLineWidth(4); // for example...
    drawVector(i_origin, i_origin + i_direction);
}


void drawSmoothTriangleMesh( Mesh const & i_mesh ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {
        Vec3 p0 = i_mesh.vertices[i_mesh.triangles[tIt][0]];
        Vec3 n0 = i_mesh.normals[i_mesh.triangles[tIt][0]];

        Vec3 p1 = i_mesh.vertices[i_mesh.triangles[tIt][1]];
        Vec3 n1 = i_mesh.normals[i_mesh.triangles[tIt][1]];

        Vec3 p2 = i_mesh.vertices[i_mesh.triangles[tIt][2]];
        Vec3 n2 = i_mesh.normals[i_mesh.triangles[tIt][2]];

        glNormal3f( n0[0] , n0[1] , n0[2] );
        glVertex3f( p0[0] , p0[1] , p0[2] );
        glNormal3f( n1[0] , n1[1] , n1[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glNormal3f( n2[0] , n2[1] , n2[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();

}

void drawTriangleMesh( Mesh const & i_mesh ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {
        Vec3 p0 = i_mesh.vertices[i_mesh.triangles[tIt][0]];
        Vec3 p1 = i_mesh.vertices[i_mesh.triangles[tIt][1]];
        Vec3 p2 = i_mesh.vertices[i_mesh.triangles[tIt][2]];

        Vec3 n = i_mesh.triangle_normals[tIt];

        glNormal3f( n[0] , n[1] , n[2] );

        glVertex3f( p0[0] , p0[1] , p0[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();

}

void drawMesh( Mesh const & i_mesh ){
    if(display_smooth_normals)
        drawSmoothTriangleMesh(i_mesh) ;
    else {
        drawTriangleMesh(i_mesh) ;
    }
}

void drawVectorField( std::vector<Vec3> const & i_positions, std::vector<Vec3> const & i_directions ) {
    glLineWidth(1.);
    for(unsigned int pIt = 0 ; pIt < i_directions.size() ; ++pIt) {
        Vec3 to = i_positions[pIt] + 0.02*i_directions[pIt];
        drawVector(i_positions[pIt], to);
    }
}

void drawNormals(Mesh const& i_mesh){

    if(display_smooth_normals){
        drawVectorField( i_mesh.vertices, i_mesh.normals );
    } else {
        std::vector<Vec3> triangle_baricenters;
        for ( const Triangle& triangle : i_mesh.triangles ){
            Vec3 triangle_baricenter (0.,0.,0.);
            for( unsigned int i = 0 ; i < 3 ; i++ )
                triangle_baricenter += i_mesh.vertices[triangle[i]];
            triangle_baricenter /= 3;
            triangle_baricenters.push_back(triangle_baricenter);
        }

        drawVectorField( triangle_baricenters, i_mesh.triangle_normals );
    }
}
void drawPointSet( std::vector< Vec3 > const & i_positions , std::vector< Vec3 > const & i_normals ) {
    glBegin(GL_POINTS);
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        glNormal3f( i_normals[pIt][0] , i_normals[pIt][1] , i_normals[pIt][2] );
        glVertex3f( i_positions[pIt][0] , i_positions[pIt][1] , i_positions[pIt][2] );
    }
    glEnd();
}

//Draw fonction
void draw () {


        glColor3f(0.8,1,0.8);
        drawMesh(mesh);

        if(display_normals){
            glColor3f(1.,0.,0.);
            drawNormals(mesh);
        }
}

void drawQuadrillage(){
    glPointSize(2); 
    glColor3f(0.8,0.8,0);
    drawPointSet( grid.quadrillage, grid.quadrillage);
}

void drawCelluleNonVide(){
    glPointSize(5); 
    glColor3f(0.9,0.2,0.6);

    glBegin(GL_POINTS);
    for(unsigned int pIt = 0 ; pIt < grid.quadrillage.size() ; ++pIt) {
        if( grid.cellules[pIt].pointInCellule.size() > 0){

        glNormal3f( grid.quadrillage[pIt][0] , grid.quadrillage[pIt][1] ,grid.quadrillage[pIt][2] );
        glVertex3f( grid.quadrillage[pIt][0] , grid.quadrillage[pIt][1] , grid.quadrillage[pIt][2] );
    }
    }
    glEnd();
}


void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    drawQuadrillage();
    drawCelluleNonVide();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    glutPostRedisplay ();
}

// ------------------------------------
// User inputs
// ------------------------------------
//Keyboard event
void key (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;


    case 'w':
        GLint polygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, polygonMode);
        if(polygonMode[0] != GL_FILL)
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;

    case 'n': //Press n key to display normals
        display_normals = !display_normals;
        break;


    case 's': //Switches between face normals and vertices normals
        display_smooth_normals = !display_smooth_normals;
        break;

    default:
        break;
    }
    idle ();
}

//Mouse events
void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle ();
}

//Mouse motion, update camera
void motion (int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate (x, y);
    }
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH), (lastY-y)/static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}




void reshape(int w, int h) {
    camera.resize (w, h);
}










void remplirCellule(Grid &grid, std::vector<Vec3>const & positions , std::vector<Vec3>const & normals){
    float Xmin = grid.BBmin[0];
    float Ymin = grid.BBmin[1];
    float Zmin = grid.BBmin[2];

    float Xmax = grid.BBmax[0];
    float Ymax = grid.BBmax[1];
    float Zmax = grid.BBmax[2];

    float intervalX=(Xmax-Xmin)/(float)(grid.x-1);
    float intervalY=(Ymax-Ymin)/(float)(grid.y-1);
    float intervalZ=(Zmax-Zmin)/(float)(grid.z-1);
    
    for( int i = 0; i< positions.size(); i++){
        int x = (positions[i][0]-grid.BBmin[0])/intervalX;
        int y = (positions[i][1]-grid.BBmin[1])/intervalY;
        int z = (positions[i][2]-grid.BBmin[2])/intervalZ;
        int voxelIndice = x*grid.y*grid.z + y* grid.z + z;
        printf(" initCoord    %f %f %f \n",positions[i][0],positions[i][1],positions[i][2]);
        printf(" test cube %i \n", x*grid.y*grid.z + y* grid.z + z);
        grid.voxels[voxelIndice].pointInCellule.push_back(positions[i]);
//        printf("         %f %f %f \n\n ", grid.quadrillage[grid.voxels[voxelIndice].cube[0]][0], grid.quadrillage[grid.voxels[voxelIndice].cube[0]][1], grid.quadrillage[grid.voxels[voxelIndice].cube[0]][2]);
        printf("         %f %f %f \n\n ", grid.quadrillage[voxelIndice][0], grid.quadrillage[voxelIndice][1], grid.quadrillage[voxelIndice][2]);

       grid.cellules[voxelIndice].pointInCellule.push_back(positions[i]);
        
    }
}

std::vector< Triangle > triangleCellule(Grid &grid, std::vector<Vec3>const & positions , std::vector< Triangle > & triangles){
    float Xmin = grid.BBmin[0];
    float Ymin = grid.BBmin[1];
    float Zmin = grid.BBmin[2];

    float Xmax = grid.BBmax[0];
    float Ymax = grid.BBmax[1];
    float Zmax = grid.BBmax[2];
    std::vector< Triangle > newTriangles;
    
    float intervalX=(Xmax-Xmin)/(float)(grid.x-1);
    float intervalY=(Ymax-Ymin)/(float)(grid.y-1);
    float intervalZ=(Zmax-Zmin)/(float)(grid.z-1);


    for( int i=0; i< triangles.size(); i++){
        int x0= (triangles[i].v[0][0]-grid.BBmin[0])/intervalX;
        int y0 = (triangles[i].v[0][1]-grid.BBmin[1])/intervalY;
        int z0 = (triangles[i].v[0][2]-grid.BBmin[2])/intervalZ;

        int voxelIndice0 = x0*grid.y0*grid.z + y0* grid.z + z0;

        int x1= (triangles[i].v[1][0]-grid.BBmin[0])/intervalX;
        int y1 = (triangles[i].v[1][1]-grid.BBmin[1])/intervalY;
        int z1 = (triangles[i].v[1][2]-grid.BBmin[2])/intervalZ;

        int voxelIndice1 = x1*grid.y*grid.z + y1* grid.z + z1;

        int x2= (triangles[i].v[2][0]-grid.BBmin[0])/intervalX;
        int y2 = (triangles[i].v[2][1]-grid.BBmin[1])/intervalY;
        int z2 = (triangles[i].v[2][2]-grid.BBmin[2])/intervalZ;

        int voxelIndice2 = x2*grid.y*grid.z + y2* grid.z + z2;
        if(voxelIndice0 != voxelIndice1 && voxelIndice0 != voxelIndice2 && voxelIndice1 != voxelIndice2)
            newTriangles.push_back(triangles[i]);

    }
    return newTriangles;

}











// ------------------------------------
// Start of graphical application
// ------------------------------------
int main (int argc, char ** argv) {
    if (argc > 2) {
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("TP HAI714I");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);

    //Unit sphere mesh loaded with precomputed normals
    openOFF("data/elephant.off", mesh.vertices, mesh.normals, mesh.triangles);

    mesh.computeNormals();

    initBB(grid,mesh.vertices);
    initVoxel(grid);


    initQuadrillage(grid);



    remplirCellule(grid, mesh.vertices , mesh.normals);
    //mesh.triangles = 
    triangleCellule(grid, mesh.vertices , mesh.triangles);


    glutMainLoop ();
    return EXIT_SUCCESS;
}

