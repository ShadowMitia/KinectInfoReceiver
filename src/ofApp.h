#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#include <vector>

#define HOST "localhost"
#define PORT 12345

class testApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();

    void mousePressed(int x, int y, int button);

    ofxKinect movie, movie2;
	ofxCvGrayscaleImage movieDepth, movieDepth2;

	ofEasyCam cam;

    
    ofxCvColorImage rgb,hsb, rgb2, hsb2;
	ofxCvGrayscaleImage hue,sat,bri,filtered, hue2, sat2, bri2, filtered2;    
    ofxCvContourFinder contours, contours2;
    
    int w,h;
    int findHue;

	int resize;


	ofMesh racket1Mesh;

	float kinect1CentroidX;
	float kinect1CentroidY;
	float kinect1CentroidZ;

	float kinect2CentroidX;
	float kinect2CentroidY;
	float kinect2CentroidZ;


	float racket1Angle;
	float racket2Angle;


	ofPoint top, bottom, left, right;
	float width, height;



	ofxOscSender sender;
	

};

#endif