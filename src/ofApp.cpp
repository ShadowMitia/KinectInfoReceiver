#include "ofApp.h"

//--------------------------------------------------------------
void testApp::setup() {
    ofBackground(0,0,0);
    
    
	w = kinect1.getWidth();
	h = kinect1.getHeight();

	resize = 4;

    //reserve memory for cv images
	hsb.allocate(w, h);
	hue.allocate(w / resize, h / resize);
	filtered.allocate(w / resize, h / resize);

	hsb2.allocate(w, h);
	hue2.allocate(w / resize, h / resize);
	filtered2.allocate(w / resize, h / resize);

	findHue = 165;

	ofSetFrameRate(60);

	showVideoFeed = false;

	kinect1CentroidX = 0;
	kinect1CentroidY = 0;
	kinect1CentroidZ = 0;

	kinect1OldCentroidX = 0;
	kinect1OldCentroidY = 0;
	kinect1OldCentroidZ = 0;

	sender.setup(HOST, PORT);



	tiltAngle1 = 0;
}

//--------------------------------------------------------------
void testApp::update(){
    
	ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));

    kinect1.update();
    if (kinect1.isFrameNew()) {
        
		hsb.resize(w, h);
        //copy webcam pixels to rgb image
        hsb.setFromPixels(kinect1.getPixels(), w, h);
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

			width = contours.blobs[0].boundingRect.width / 2;
			height = contours.blobs[0].boundingRect.height / 2;

			/*
			top = ofVec3f(kinect1CentroidX, kinect1CentroidY - height / 2);
			bottom = ofVec3f(kinect1CentroidX, kinect1CentroidY +  height / 2 );

			left = ofVec3f(kinect1CentroidX + width / 2, kinect1CentroidY);
			right = ofVec3f(kinect1CentroidX - width / 2, kinect1CentroidY);
			*/
			
			
			int offset = 5;
			/*
			topM = ofVec3f(kinect1CentroidX , kinect1CentroidY + offset, kinect1.getDistanceAt(kinect1CentroidX * (int)(resize * 3/2), (kinect1CentroidY + offset) * (int)(resize * 3/2) ));
			bottomM = ofVec3f(kinect1CentroidX, kinect1CentroidY - offset, kinect1.getDistanceAt(kinect1CentroidX * (int)(resize * 3/2), (kinect1CentroidY - offset) * (int)(resize * 3/2)));
			leftM = ofVec3f(kinect1CentroidX + offset, kinect1CentroidY, kinect1.getDistanceAt((kinect1CentroidX + offset) * (int)(resize * 3/2) , kinect1CentroidY * (int)(resize * 3/2)));
			rightM = ofVec3f(kinect1CentroidX - offset, kinect1CentroidY, kinect1.getDistanceAt(( kinect1CentroidX - offset) * (int)(resize * 3/2), kinect1CentroidY * (int)(resize * 3/2) ));
			*/


			topM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY + offset) * (int)(resize) );
			bottomM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY - offset) * (int)(resize) );
			leftM = kinect1.getWorldCoordinateAt( (kinect1CentroidX + offset) * (int)(resize) , kinect1CentroidY * (int)(resize) );
			rightM = kinect1.getWorldCoordinateAt( ( kinect1CentroidX - offset) * (int)(resize), kinect1CentroidY * (int)(resize) );
			kinect1CentroidZ = kinect1.getDistanceAt( kinect1CentroidX * (int)(resize), kinect1CentroidY * (int)(resize));
			centroidM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), kinect1CentroidY * (int)(resize) );

			//racket1Angle = atan( (PI / 2) / 75 * (bottomM.z - topM.z));

			int off = 25;

			if ( leftM.z - rightM.z > -off && leftM.z - rightM.z < off) {
				racket1AngleVerti = leftM.z - rightM.z;

			}

			if ( topM.z - bottomM.z > -off && topM.z - bottomM.z < off) {
				racket1AngleHori = topM.z - bottomM.z;
			}

		}
    }

	
	kinect2.update();
    
	if (kinect2.isFrameNew()) {
		hsb2.resize(w, h);
        //copy webcam pixels to rgb image
        hsb2.setFromPixels(kinect2.getPixels(), w, h);
		hsb2.resize(w/resize, h / resize);

        //convert to hsb
        hsb2.convertRgbToHsv();
		
        
        //store the three channels as grayscale images
        hsb2.convertToGrayscalePlanarImage(hue2, 0);
        
        //filter image based on the hue value we're looking for
		for (int i=0; i<hue2.width*hue2.height; i++) {
			filtered2.getPixels()[i] = ofInRange(hue2.getPixels()[i], findHue - 15,  findHue + 15) ? 255 : 0;
        }
        filtered2.flagImageChanged();

        //run the contour finder on the filtered image to find blobs with a certain hue
        contours2.findContours(filtered, 50, (w/resize)*(h/resize)/2, 1, false);

		if (contours2.nBlobs > 0) {

			kinect2CentroidX = contours2.blobs[0].centroid.x;
			kinect2CentroidY = contours2.blobs[0].centroid.y;

			width = contours2.blobs[0].boundingRect.width / 2;
			height = contours2.blobs[0].boundingRect.height / 2;

			/*
			top2 = ofVec3f(kinect2CentroidX, kinect2CentroidY - height / 2);
			bottom2 = ofVec3f(kinect2CentroidX, kinect2CentroidY +  height / 2 );

			left2 = ofVec3f(kinect2CentroidX + width / 2, kinect2CentroidY);
			right2 = ofVec3f(kinect2CentroidX - width / 2, kinect2CentroidY);
			*/


		}
    }
	


	// osc stuff
	ofxOscMessage message;

	if (kinect1.isConnected()) {
		/*
		message.setAddress("/kinect1/connected");
		message.addStringArg("Kinect 1 connected : (Serial) " + kinect1.getSerial() );
		message.addIntArg(1);
		*/
		message.setAddress("/kinect1/position");
		message.addFloatArg(centroidM.x);
		message.addFloatArg(centroidM.y);
		message.addFloatArg(centroidM.z);
		message.addFloatArg(racket1AngleHori);
		message.addFloatArg(racket1AngleVerti);
		
		sender.sendMessage(message);
	} else {
		message.setAddress("/kinect1/connected");
		message.addStringArg("Kinect 1 not found ");
		message.addIntArg(0);
		sender.sendMessage(message);
	}

	if (kinect2.isConnected()){
		/*
		message.setAddress("/kinect2/connected");
		message.addStringArg("Kinect 2 connected : (Serial) " + kinect2.getSerial() );
		message.addIntArg(1);
		*/
		message.setAddress("/kinect2/position");
		message.addFloatArg(kinect2CentroidX);
		message.addFloatArg(kinect2CentroidY);
		message.addFloatArg(1);
		sender.sendMessage(message);
	} else {
		message.setAddress("/kinect2/connected");
		message.addStringArg("Kinect 2 not found " );
		message.addIntArg(0);
		sender.sendMessage(message);
	}

	
}

//--------------------------------------------------------------
void testApp::draw(){
	
    ofSetColor(255,255,255);
    
    //draw all cv images
	hsb.draw(0, 0);
    filtered.draw(w / resize, h / resize);
    hue.draw(0, h / resize);
	contours.draw(0, 0);



	ofSetColor(255,255,255);
	hsb2.draw(w, 0);
    filtered2.draw(w +w / resize, h / resize);
    hue2.draw(w, h / resize);
	contours2.draw(w, 0);

	
	ofSetColor(255, 0, 255);
	ofCircle(topM, 2);
	ofCircle(bottomM, 2);
	ofCircle(leftM, 2);
	ofCircle(rightM, 2);
	
    

	
 //   //draw red circles for found blobs
 //   for (int i=0; i<contours.nBlobs; i++) {
	//	ofCircle(contours.blobs[0].centroid.x, contours.blobs[0].centroid.y, 2);
 //   }

	////cout << "w/" << resize << ": " << w / resize <<  " " << " h/" << resize << ": " << h / resize << endl;
	//

	ofSetColor(255, 0, 0);
	stringstream output;

	
	if (contours.nBlobs){
		output.clear();
		output << "rectangle violet" << endl;
		output << "x: " << contours.blobs[0].centroid.x << " y: " << contours.blobs[0].centroid.y << endl;
		output << kinect1.getWorldCoordinateAt(contours.blobs[0].centroid.x, contours.blobs[0].centroid.y) << endl;

		output << "top: " << topM  << " bottom " << bottomM << endl;
		output << "left: " << leftM << " right " << rightM << endl;
		output << "angle hori: " << racket1AngleHori << endl;
		output << "angle verti: " << racket1AngleVerti << endl;
	}

	ofDrawBitmapString(output.str(), w+50, 400);

	/*
	if (contours.nBlobs > 0) {
		int w = 640;
		int h = 480;
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_POINTS);
		int step = 2;
		for(int y = contours.blobs[0].boundingRect.y; y < contours.blobs[0].boundingRect.height * resize; y += step) {
			for(int x = contours.blobs[0].boundingRect.x; x < contours.blobs[0].boundingRect.width * resize; x += step) {
				if(kinect1.getDistanceAt(x, y) > 0) {
					mesh.addColor(kinect1.getColorAt(x,y));
					mesh.addVertex(kinect1.getWorldCoordinateAt(x, y));

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
	
	*/
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

void testApp::keyPressed(int key) {
	switch (key) {
	case '1':
		cout << "Trying to load Kinect 1" << endl;
		kinect1.setRegistration(true);
		kinect1.init();
		kinect1.open(0);
		break;

	case '2':
		cout << "Trying to load Kinect 2" << endl;
		kinect2.setRegistration(true);
		kinect2.init();
		kinect2.open(1);
		break;

	case '4':
		cout << "Closing Kinect 1" << endl;
		kinect1.setCameraTiltAngle(0);
		kinect1.close();
		break;

	case '5':
		cout << "Closing Kinect 2" << endl;
		kinect2.setCameraTiltAngle(0);
		kinect2.close();
		break;

	case OF_KEY_F9:
		tiltAngle1++;
		if (tiltAngle1 > 30) tiltAngle1 = 30;
		kinect1.setCameraTiltAngle(tiltAngle1);
		break;

	case OF_KEY_F10:
		tiltAngle1--;
		if (tiltAngle1 < -30) tiltAngle1 = -30;
		kinect1.setCameraTiltAngle(tiltAngle1);
		break;

	case 'v':
		showVideoFeed = ! showVideoFeed;
	break;

	default:
		cout << key << " " << (char)key << endl;
		break;
	}

}



