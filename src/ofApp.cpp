#include "ofApp.h"

//--------------------------------------------------------------
void testApp::setup() {
    ofBackground(0,0,0);
    
	movie.setRegistration(true);
	movie2.setRegistration(true);

    movie.init();
	movie.open();

	movie2.init();
	movie2.open();
    
	w = kinect1.getWidth();
	h = kinect1.getHeight();

	resize = 4;

    //reserve memory for cv images
	hueKinect1.allocate(w, h);
	filteredKinect1.allocate(w, h);
	hueKinect2.allocate(w, h);
	filteredKinect2.allocate(w, h);

	findHue = 165;

	ofSetFrameRate(60);
	drawDebug = false;
	showVideoFeed = false;

	oldCentroidX = 0;
	oldCentroidY = 0;
	oldCentroidZ = 0;

	centroidX = 0;
	centroidY = 0;
	centroidZ = 0;

	xMov = 0;
	yMov = 0;
	zMov = 0;


	sender.setup(HOST, PORT);
}

//--------------------------------------------------------------
void testApp::update(){
    
	ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));

    movie.update();
    if (movie.isFrameNew()) {
        
		hsb.resize(w, h);
        //copy webcam pixels to rgb image
        hsb.setFromPixels(movie.getPixels(), w, h);
		hsb.resize(w/resize, h / resize);

        //convert to hsb
        hsb.convertRgbToHsv();
		
        
        //store the three channels as grayscale images
        hsb.convertToGrayscalePlanarImage(hue, 0);
        
        //filter image based on the hue value we're looking for
		for (int i=0; i<hue.width*hue.height; i++) {
			filtered.getPixels()[i] = ofInRange(hue.getPixels()[i], findHue - 15,  findHue + 15) ? 255 : 0;
        }
        filtered.flagImageChanged();

        //run the contour finder on the filtered image to find blobs with a certain hue
        contours.findContours(filtered, 50, (w/resize)*(h/resize)/2, 1, false);

		if (contours.nBlobs > 0) {

			kinect1CentroidX = contours.blobs[0].centroid.x;
			kinect1CentroidY = contours.blobs[0].centroid.y;

			//centroidZ = movie.getDistanceAt(centroidX, centroidY) + (height / 2);

			//centroidZ = movie.getDistanceAt(centroidX, centroidY) + (400 / 2);


			width = contours.blobs[0].boundingRect.width / 2;
			height = contours.blobs[0].boundingRect.height / 2;

			top = ofVec3f(kinect1CentroidX, kinect1CentroidY - height / 2);
			bottom = ofVec3f(kinect1CentroidX, kinect1CentroidY +  height / 2 );

			left = ofVec3f(kinect1CentroidX + width / 2, kinect1CentroidY);
			right = ofVec3f(kinect1CentroidX - width / 2, kinect1CentroidY);
		}
    }

	/*
	movie2.update();
    if (movie2.isFrameNew()) {
        
		hsb2.resize(w, h);
        //copy webcam pixels to rgb image
        hsb2.setFromPixels(movie2.getPixels(), w, h);
		hsb2.resize(w/resize, h / resize);

		
		//convert to hsb2
        hsb2.convertRgbToHsv();
		
        
        //store the three channels as grayscale images
        hsb2.convertToGrayscalePlanarImage(hue2, 0);
        
        //filter image based on the hue2 value we're looking for
		for (int i=0; i<hue2.width*hue2.height; i++) {
			filtered2.getPixels()[i] = ofInRange(hue2.getPixels()[i], findHue - 5, findHue + 5) ? 255 : 0;
        }
        filtered2.flagImageChanged();

        //run the contour finder on the filtered2 image to find blobs with a certain hue2
        contours2.findContours(filtered2, 50, (w/resize)*(h/resize)/2, 1, false);
    }
	*/


	// osc stuff
	ofxOscMessage message;

	if (movie.isConnected()){
		message.setAddress("/kinect1/connected");
		message.addStringArg("Kinect 1 connected : (Serial) " + movie.getSerial() );
		message.addIntArg(1);
		message.setAddress("/kinect1/position");
		message.addFloatArg(kinect1CentroidX);
		message.addFloatArg(kinect1CentroidY);
		message.addFloatArg(movie.getDistanceAt(kinect1CentroidX, kinect1CentroidY));
	} else {
		message.setAddress("/kinect1/connected");
		message.addStringArg("Kinect 1 not found " + movie.getSerial() );
		message.addIntArg(0);
	}

	sender.sendMessage(message);
}

//--------------------------------------------------------------
void testApp::draw(){
    ofSetColor(255,255,255);
    
    //draw all cv images
	hsb.draw(0, 0);
    filtered.draw(w/resize, 0);
    hue.draw(0,h/resize);
	contours.draw(0, 0);


	ofSetColor(255, 0, 255);
	ofCircle(top, 5);
	ofCircle(bottom, 5);
	ofCircle(left, 5);
	ofCircle(right, 5);

	//racket1Mesh.draw();
	/*
	hsb2.draw(w, 0);
    filtered2.draw(w + w/resize, 0);
    hue2.draw(w,h/resize);
	contours2.draw(w, 0);
	*/
    
    ofSetColor(255, 0, 0);
    ofFill();
    
	
    //draw red circles for found blobs
    for (int i=0; i<contours.nBlobs; i++) {
		ofCircle(contours.blobs[0].centroid.x, contours.blobs[0].centroid.y, 2);
    }

	//cout << "w/" << resize << ": " << w / resize <<  " " << " h/" << resize << ": " << h / resize << endl;


	ofSetColor(255, 0, 0);
			stringstream output;
			if (contours.nBlobs){
				output.clear();
				output << "rectangle violet" << endl;
				output << "x: " << contours.blobs[0].centroid.x << " y: " << contours.blobs[0].centroid.y << endl;
				output << movie.getWorldCoordinateAt(contours.blobs[0].centroid.x, contours.blobs[0].centroid.y) << endl;
				
				output << "left: " << left << " right: " << right << endl;
				output <<  movie.getWorldCoordinateAt(left.x, left.y) << " =  " << movie.getWorldCoordinateAt(right.x, right.y) << endl;
				ofVec3f tmp = movie.getWorldCoordinateAt(right.x, right.y);
				ofVec3f tmp2 = movie.getWorldCoordinateAt(left.x, left.y);
				output << "delta: " << tmp.z - tmp2.z << endl;
			}
			ofDrawBitmapString(output.str(), w+50, 400);
			if (contours.nBlobs > 0) {
				int w = 640;
				int h = 480;
				ofMesh mesh;
				mesh.setMode(OF_PRIMITIVE_POINTS);
				int step = 2;
				for(int y = contours.blobs[0].boundingRect.y; y < contours.blobs[0].boundingRect.height * resize; y += step) {
					for(int x = contours.blobs[0].boundingRect.x; x < contours.blobs[0].boundingRect.width * resize; x += step) {
						if(movie.getDistanceAt(x, y) > 0) {
							mesh.addColor(movie.getColorAt(x,y));
							mesh.addVertex(movie.getWorldCoordinateAt(x, y));

						}
					}
				}
				cam.begin();
				glPointSize(3);
				ofPushMatrix();
				// the projected points are 'upside down' and 'backwards' 
				ofScale(1, -1, -1);
				ofTranslate(0, 0, -1000); // center the points a bit
				ofEnableDepthTest();
				mesh.drawVertices();
				ofDisableDepthTest();
				ofPopMatrix();
				cam.end();
			}
	

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
	if (button == OF_MOUSE_BUTTON_RIGHT){
    //calculate local mouse x,y in image
    int mx = x % w / resize;
    int my = y % h / resize;
    
    //get hue value on mouse position
    findHue = hue.getPixels()[my*(int)w/resize+mx];

	cout << "hue: " << findHue << endl;
	}
}