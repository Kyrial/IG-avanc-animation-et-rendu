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
#include <math.h>

#include <algorithm>
#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"
#include "src/jmkdtree.h"




std::vector< Vec3 > positions;
std::vector< Vec3 > normals;

std::vector< Vec3 > positions2;
std::vector< Vec3 > normals2;

Vec3 Centroide1= Vec3(-10,-10,-10);
Vec3 Centroide2= Vec3(-10,-10,-10);

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 640;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static bool fullScreen = false;


void HPSS( Vec3 inputPoint , Vec3 & outputPoint , Vec3 & outputNormal ,
std::vector<Vec3>const & positions , std::vector<Vec3>const & normals , BasicANNkdTree const & kdtree ,
int kernel_type, unsigned int nbIterations,unsigned int knn);




// ------------------------------------------------------------------------------------------------------------
// i/o and some stuff
// ------------------------------------------------------------------------------------------------------------
void loadPN (const std::string & filename , std::vector< Vec3 > & o_positions , std::vector< Vec3 > & o_normals ) {
    unsigned int surfelSize = 6;
    FILE * in = fopen (filename.c_str (), "rb");
    if (in == NULL) {
        std::cout << filename << " is not a valid PN file." << std::endl;
        return;
    }
    size_t READ_BUFFER_SIZE = 1000; // for example...
    float * pn = new float[surfelSize*READ_BUFFER_SIZE];
    o_positions.clear ();
    o_normals.clear ();
    while (!feof (in)) {
        unsigned numOfPoints = fread (pn, 4, surfelSize*READ_BUFFER_SIZE, in);
        for (unsigned int i = 0; i < numOfPoints; i += surfelSize) {
            o_positions.push_back (Vec3 (pn[i], pn[i+1], pn[i+2]));
            o_normals.push_back (Vec3 (pn[i+3], pn[i+4], pn[i+5]));
        }

        if (numOfPoints < surfelSize*READ_BUFFER_SIZE) break;
    }
    fclose (in);
    delete [] pn;
}
void savePN (const std::string & filename , std::vector< Vec3 > const & o_positions , std::vector< Vec3 > const & o_normals ) {
    if ( o_positions.size() != o_normals.size() ) {
        std::cout << "The pointset you are trying to save does not contain the same number of points and normals." << std::endl;
        return;
    }
    FILE * outfile = fopen (filename.c_str (), "wb");
    if (outfile == NULL) {
        std::cout << filename << " is not a valid PN file." << std::endl;
        return;
    }
    for(unsigned int pIt = 0 ; pIt < o_positions.size() ; ++pIt) {
        fwrite (&(o_positions[pIt]) , sizeof(float), 3, outfile);
        fwrite (&(o_normals[pIt]) , sizeof(float), 3, outfile);
    }
    fclose (outfile);
}
void scaleAndCenter( std::vector< Vec3 > & io_positions ) {
    Vec3 bboxMin( FLT_MAX , FLT_MAX , FLT_MAX );
    Vec3 bboxMax( FLT_MIN , FLT_MIN , FLT_MIN );
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        for( unsigned int coord = 0 ; coord < 3 ; ++coord ) {
            bboxMin[coord] = std::min<float>( bboxMin[coord] , io_positions[pIt][coord] );
            bboxMax[coord] = std::max<float>( bboxMax[coord] , io_positions[pIt][coord] );
        }
    }
    Vec3 bboxCenter = (bboxMin + bboxMax) / 2.f;
    float bboxLongestAxis = std::max<float>( bboxMax[0]-bboxMin[0] , std::max<float>( bboxMax[1]-bboxMin[1] , bboxMax[2]-bboxMin[2] ) );
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        io_positions[pIt] = (io_positions[pIt] - bboxCenter) / bboxLongestAxis;
    }
}

void applyRandomRigidTransformation( std::vector< Vec3 > & io_positions , std::vector< Vec3 > & io_normals ) {
    srand(time(NULL));
    Mat3 R = Mat3::RandRotation();
    Vec3 t = Vec3::Rand(1.f);
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        io_positions[pIt] = R * io_positions[pIt] + t;
        io_normals[pIt] = R * io_normals[pIt];
    }
}

void subsample( std::vector< Vec3 > & i_positions , std::vector< Vec3 > & i_normals , float minimumAmount = 0.1f , float maximumAmount = 0.2f ) {
    std::vector< Vec3 > newPos , newNormals;
    std::vector< unsigned int > indices(i_positions.size());
    for( unsigned int i = 0 ; i < indices.size() ; ++i ) indices[i] = i;
    srand(time(NULL));
    std::random_shuffle(indices.begin() , indices.end());
    unsigned int newSize = indices.size() * (minimumAmount + (maximumAmount-minimumAmount)*(float)(rand()) / (float)(RAND_MAX));
    newPos.resize( newSize );
    newNormals.resize( newSize );
    for( unsigned int i = 0 ; i < newPos.size() ; ++i ) {
        newPos[i] = i_positions[ indices[i] ];
        newNormals[i] = i_normals[ indices[i] ];
    }
    i_positions = newPos;
    i_normals = newNormals;
}

bool save( const std::string & filename , std::vector< Vec3 > & vertices , std::vector< unsigned int > & triangles ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl;

    unsigned int n_vertices = vertices.size() , n_triangles = triangles.size()/3;
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << vertices[v][0] << " " << vertices[v][1] << " " << vertices[v][2] << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << triangles[3*f] << " " << triangles[3*f+1] << " " << triangles[3*f+2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}



// ------------------------------------------------------------------------------------------------------------
// rendering.
// ------------------------------------------------------------------------------------------------------------

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
    glEnable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
}



void drawTriangleMesh( std::vector< Vec3 > const & i_positions , std::vector< unsigned int > const & i_triangles ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_triangles.size() / 3 ; ++tIt) {
        Vec3 p0 = i_positions[3*tIt];
        Vec3 p1 = i_positions[3*tIt+1];
        Vec3 p2 = i_positions[3*tIt+2];
        Vec3 n = Vec3::cross(p1-p0 , p2-p0);
        n.normalize();
        glNormal3f( n[0] , n[1] , n[2] );
        glVertex3f( p0[0] , p0[1] , p0[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();
}

void drawPointSet( std::vector< Vec3 > const & i_positions , std::vector< Vec3 > const & i_normals ) {
    glBegin(GL_POINTS);
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        glNormal3f( i_normals[pIt][0] , i_normals[pIt][1] , i_normals[pIt][2] );
        glVertex3f( i_positions[pIt][0] , i_positions[pIt][1] , i_positions[pIt][2] );
    }
    glEnd();
}


void drawCentroide () {
    glPointSize(20); // for example...
glBegin(GL_POINTS);
    glColor3f(0.8,1.0,0.4);
    glNormal3f( Centroide1[0] , Centroide1[1] ,Centroide1[2] );
    glVertex3f( Centroide1[0] , Centroide1[1] , Centroide1[2] );
glEnd();
glBegin(GL_POINTS);
glColor3f(1,0.1,0.5);
    glNormal3f( Centroide2[0] , Centroide2[1] ,Centroide2[2] );
    glVertex3f( Centroide2[0] , Centroide2[1] , Centroide2[2] );
     glEnd();
}


void draw () {
    glPointSize(2); // for example...

    
    drawPointSet(positions , normals);

    glColor3f(1,0.5,0.5);
    drawPointSet(positions2 , normals2);
}








void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();

drawCentroide ();


    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    glutPostRedisplay ();
}

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

    default:
        break;
    }
    idle ();
}

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



void projection(Vec3 input , Vec3 & output ,Vec3 const & position , Vec3 const & normal){

    float X  = Vec3::dot(  ( input - position ) , normal) / normal.length();
    output = input -  X* normal;

}

void APSS( Vec3 inputPoint , Vec3 & outputPoint , Vec3 & outputNormal ,
std::vector<Vec3>const & positions , std::vector<Vec3>const & normals , BasicANNkdTree const & kdtree ,
int kernel_type, unsigned int nbIterations = 1 , unsigned int knn = 10 ) {


}




void HPSS( Vec3 inputPoint , Vec3 & outputPoint , Vec3 & outputNormal ,
std::vector<Vec3>const & positions , std::vector<Vec3>const & normals , BasicANNkdTree const & kdtree ,
int kernel_type, unsigned int nbIterations = 1 , unsigned int knn = 10 ) {


int k=0;
while(k<nbIterations){

ANNidxArray id_nearest_neighbors =new ANNidx[ knn ];
ANNdistArray square_distances_to_neighbors = new ANNdist[ knn ];




//knearest( point_t const & i_position , int k , ANNidxArray id_nearest_neighbors , ANNdistArray square_distances_to_neighbors )
kdtree.knearest( inputPoint , knn , id_nearest_neighbors, square_distances_to_neighbors );
//Vec3 input =inputPoint;
Vec3 n_nomi = Vec3(0,0,0);
Vec3 c_nomi=  Vec3(0,0,0);
float sumW=0;

Vec3 output[knn];

for( int i=0; i<knn; i++){

    projection(inputPoint, output[i], positions[id_nearest_neighbors[i]], normals[id_nearest_neighbors[i]]);
    




    float h =  sqrt(square_distances_to_neighbors[knn-1]);
    double w=0;

    double r = (inputPoint - positions[id_nearest_neighbors[i]]).length();

    if (kernel_type==0){
        w=exp(-pow(r,2)/pow(h,2));
    }
        if (kernel_type==1){
        w=pow(1- ( r / h ) ,4) * ( 1 + 4 * ( r / h ) );
    }
    if (kernel_type==2){
        w=pow(h/r,2);
    }
    //printf("w = %f  et h = %f\n",w, h);
    c_nomi += w*output[i];
    n_nomi += (w*normals[id_nearest_neighbors[i]]);
    sumW += w;

}
    outputPoint =  c_nomi / sumW;
    outputNormal = n_nomi / sumW;
  delete [] id_nearest_neighbors;
  delete [] square_distances_to_neighbors;
  k++;
  inputPoint = outputPoint;

}
}


void centroide(std::vector<Vec3>const & position,  Vec3 & centroide)
{
    
    //Vec3 normalCentroide= Vec3(0.0,0.0,0.0);

    for(int i =0; i< position.size();i++){
        centroide+=position[i];
    }
    centroide= (1.0/position.size())*centroide;

    printf(" ll %f, %f, %f \n",centroide[0],centroide[1],centroide[2] );
}


void translation(std::vector<Vec3>& position,  Vec3 & source, Vec3 & target)
{
    Vec3 translation = target - source;

    for(int i =0; i< position.size();i++){
        position[i]+= translation;
    }
    

    //printf(" ll %f, %f, %f \n",centroide[0],centroide[1],centroide[2] );
}





//1_ calculs du centroide
//2_ translate source sur target
//3_ rotation aleatoire (ou l'Identité)
//2_ et 3_ -> calculs de l'initialisation = ACP -> recallé les axes (ou pas) # init
//
//ICP ->
//S = P. Qi
// (position - le centroide -> dans la matrice)
// remplir une matrice 3*3 par le produit de ca
//S.setRotation


void PositionToCentroide(std::vector<Vec3> & position, std::vector<Vec3> & out,Vec3 & target) {
     out.resize( position.size() );
    for(int i =0; i< position.size();i++){
        Vec3 translation = target-position[i];
        out[i]= translation;
       // printf("lala %f  \n",position[i][0]);
    }
}





void ICP( std::vector<Vec3>  &ps , std::vector<Vec3> const &nps ,
            std::vector<Vec3>  &qs , std::vector<Vec3> const & nqs ,
            BasicANNkdTree const & qsKdTree , Mat3 & rotation , Vec3 & translation , 
            unsigned int nIterations ) {

    int ite=0;
while(ite++<nIterations){

        centroide(ps, Centroide1);
        centroide(qs, Centroide2);

       // translation(ps,  Centroide1, Centroide2); 
       // Centroide1 = Centroide2;
        std::vector<Vec3> pslocal;
        std::vector<Vec3> qslocal;
        PositionToCentroide( ps, pslocal , Centroide2);

        std::vector<Vec3> psNearest;
        psNearest.resize( ps.size() );
        for(int i =0; i< ps.size(); i++){
            psNearest[i]=ps[qsKdTree.nearest(ps[i])];
        }


        PositionToCentroide( psNearest,  qslocal,Centroide2);


        Mat3 S = Mat3();

        for(int i =0; i<3; i++) 
            for( int j=0; j<3; j++){   
                float val=0; 
                for(int k = 0; k< pslocal.size(); k++){
                    val += pslocal[k][i]*qslocal[k][j];
                    //  printf("lala %f  %f\n",val ,pslocal[k][i],qslocal[k][j]);
                }
            S(i,j)= val;
            printf("meow %f \n",val);
            }


        S.setRotation();


        for(int i =0; i< ps.size(); i++){
            ps[i] = Centroide2 + S * (ps[i] - Centroide1);
        }

        }
}



int main (int argc, char ** argv) {
    if (argc > 2) {
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("j'aime les chats !!");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);


    

        // Load a first pointset, and build a kd-tree:
        //loadPN("pointsets/igea.pn" , positions , normals);
      //  loadPN("pointsets/dino_subsampled_extreme.pn" , positions , normals);
    loadPN("pointsets/igea.pn" , positions , normals);
    loadPN("pointsets/igea2.pn" , positions2 , normals2);

    srand (time(NULL));
    Mat3 ICProtation = Mat3::RandRotation();
    Vec3 ICPtranslation = Vec3(-1.0+2.0*((double)(rand())/(double)(RAND_MAX)),-1.0+2.0*((double)(rand())/(double)(RAND_MAX)),-1.0+2.0*((double)(rand())/(double)(RAND_MAX)));

    for (unsigned int pIt = 0; pIt < positions2.size() ; ++pIt){
        positions2[pIt] = ICProtation * positions2[pIt] + ICPtranslation;
        normals2[pIt] = ICProtation * normals2[pIt];
    }


        BasicANNkdTree kdtree;
        kdtree.build(positions);

               BasicANNkdTree qsdKdTree;
        qsdKdTree.build(positions2); 

     Mat3 rotation = Mat3();
     Vec3 translation = Vec3();
     int nIterations = 10;
         ICP( positions , normals ,
            positions2 , normals2 ,
            qsdKdTree ,  rotation , translation , 
             nIterations );

   //     Vec3 outputPoint;
      //  Vec3 outputNormal;
    //    for(int i=0; i< positions2.size(); i++ ){
            //HPSS( positions2[i] , positions2[i] , normals2[i] ,positions , normals , kdtree ,  0 , 30 , 20 );  
    //        HPSS( positions2[i] , positions2[i] , normals2[i] ,positions , normals , kdtree ,  0 , 5 , 8 );       
             
        // PROJECT USING MLS (HPSS and APSS):
        // TODO
        
    



    glutMainLoop ();
    return EXIT_SUCCESS;
}

