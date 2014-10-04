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

	kinect2CentroidX = 0;
	kinect2CentroidY = 0;
	kinect2CentroidZ = 0;

	kinect2OldCentroidX = 0;
	kinect2OldCentroidY = 0;
	kinect2OldCentroidZ = 0;

	sender.setup(HOST, PORT);


	//moteur kinect
	tiltAngle1 = 0;
	tiltAngle2 = 0;

	offset = 5;

}

//--------------------------------------------------------------
void testApp::update(){
    
	ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));


	//////////// KINECT 1
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
			
			
			if (kinect1CentroidY - offset < 0) {
				topM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY) * (int)(resize) );
			}
			else{
				topM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY - offset) * (int)(resize) );
			}

			if (kinect1CentroidY + offset > h / resize) {
				bottomM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY) * (int)(resize) );
			} else {
				bottomM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY + offset) * (int)(resize) );
			}

			if (kinect1CentroidX - offset < 0) {
				leftM = kinect1.getWorldCoordinateAt( (kinect1CentroidX) * (int)(resize) , kinect1CentroidY * (int)(resize) );
			} else {
				leftM = kinect1.getWorldCoordinateAt( (kinect1CentroidX - offset) * (int)(resize) , kinect1CentroidY * (int)(resize) );
			}

			if (kinect1CentroidX + offset > w / 4) {
				rightM = kinect1.getWorldCoordinateAt( ( kinect1CentroidX) * (int)(resize), kinect1CentroidY * (int)(resize) );
			} else {
				rightM = kinect1.getWorldCoordinateAt( ( kinect1CentroidX + offset) * (int)(resize), kinect1CentroidY * (int)(resize) );
			}

			/*
			topM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY + offset) * (int)(resize) );
			bottomM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), (kinect1CentroidY - offset) * (int)(resize) );
			leftM = kinect1.getWorldCoordinateAt( (kinect1CentroidX + offset) * (int)(resize) , kinect1CentroidY * (int)(resize) );
			rightM = kinect1.getWorldCoordinateAt( ( kinect1CentroidX - offset) * (int)(resize), kinect1CentroidY * (int)(resize) );
			kinect1CentroidZ = kinect1.getDistanceAt( kinect1CentroidX * (int)(resize), kinect1CentroidY * (int)(resize));

			*/
			centroidM = kinect1.getWorldCoordinateAt( kinect1CentroidX * (int)(resize), kinect1CentroidY * (int)(resize) );
			
			

			//racket1Angle = atan( (PI / 2) / 75 * (bottomM.z - topM.z));

			int off = 25;

			if ( leftM.z - rightM.z > -off && leftM.z - rightM.z < off) {
				racket1AngleVerti = leftM.z - rightM.z;

			}

			if ( topM.z - bottomM.z > -off && topM.z - bottomM.z < off) {
				racket1AngleHori = topM.z - bottomM.z;
			}

			if (centroidM.z < 400) {
				centroidM.z = 400;
			}

			// modif centre kinect 01
			centroidM.y -= 240;
			if(centroidM.y==0){ // condition pour no detect
				centroidM.y=0;
			}
			


		}
    }

	//////////// KINECT 2
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
        contours2.findContours(filtered2, 50, (w/resize)*(h/resize)/2, 1, false);

		if (contours2.nBlobs > 0) {

			kinect2CentroidX = contours2.blobs[0].centroid.x;
			kinect2CentroidY = contours2.blobs[0].centroid.y;

			width = contours2.blobs[0].boundingRect.width / 2;
			height = contours2.blobs[0].boundingRect.height / 2;
			

			if (kinect2CentroidY - offset < 0) {
				topM = kinect2.getWorldCoordinateAt( kinect2CentroidX * (int)(resize), (kinect2CentroidY) * (int)(resize) );
			}
			else{
				topM = kinect2.getWorldCoordinateAt( kinect2CentroidX * (int)(resize), (kinect2CentroidY - offset) * (int)(resize) );
			}

			if (kinect2CentroidY + offset > h / resize) {
				bottomM = kinect2.getWorldCoordinateAt( kinect2CentroidX * (int)(resize), (kinect2CentroidY) * (int)(resize) );
			} else {
				bottomM = kinect2.getWorldCoordinateAt( kinect2CentroidX * (int)(resize), (kinect2CentroidY + offset) * (int)(resize) );
			}

			if (kinect2CentroidX - offset < 0) {
				leftM = kinect2.getWorldCoordinateAt( (kinect2CentroidX) * (int)(resize) , kinect2CentroidY * (int)(resize) );
			} else {
				leftM = kinect2.getWorldCoordinateAt( (kinect2CentroidX - offset) * (int)(resize) , kinect2CentroidY * (int)(resize) );
			}

			if (kinect2CentroidX + offset > w / 4) {
				rightM = kinect2.getWorldCoordinateAt( ( kinect2CentroidX) * (int)(resize), kinect2CentroidY * (int)(resize) );
			} else {
				rightM = kinect2.getWorldCoordinateAt( ( kinect2CentroidX + offset) * (int)(resize), kinect2CentroidY * (int)(resize) );
			}

			centroid2M = kinect2.getWorldCoordinateAt( kinect2CentroidX * (int)(resize), kinect2CentroidY * (int)(resize) );



			//racket1Angle = atan( (PI / 2) / 75 * (bottomM.z - topM.z));

			int off = 25;

			if ( leftM.z - rightM.z > -off && leftM.z - rightM.z < off) {
				racket2AngleVerti = leftM.z - rightM.z;

			}

			if ( topM.z - bottomM.z > -off && topM.z - bottomM.z < off) {
				racket2AngleHori = topM.z - bottomM.z;
			}

			if (centroid2M.z < 400) {
				centroid2M.z = 400;
			}

			// modif centre kinect 02
			centroid2M.y -= 240;
			
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

		if (!centroidM.x == 0 && !centroidM.y == 0 && !centroidM.z == 0)  {
		message.setAddress("/kinect1/position");
		message.addFloatArg(centroidM.x);
		message.addFloatArg(centroidM.y);
		message.addFloatArg(centroidM.z);
		message.addFloatArg(racket1AngleHori);
		message.addFloatArg(racket1AngleVerti);
		}
		sender.sendMessage(message);
	} else {
		message.setAddress("/kinect1/connected");
		message.addStringArg("Kinect 1 not found ");
		message.addIntArg(0);
		sender.sendMessage(message);
	}

	ofxOscMessage message2;

	if (kinect2.isConnected()) {
		/*
		message.setAddress("/kinect2/connected");
		message.addStringArg("Kinect 2 connected : (Serial) " + kinect2.getSerial() );
		message.addIntArg(1);
		*/
		if (!centroid2M.x == 0 && !centroid2M.y == 0 && !centroid2M.z == 0)  {
		message2.setAddress("/kinect2/position");
		message2.addFloatArg(centroid2M.x);
		message2.addFloatArg(centroid2M.y);
		message2.addFloatArg(centroid2M.z);
		message2.addFloatArg(racket2AngleHori);
		message2.addFloatArg(racket2AngleVerti);
		}
		
		sender.sendMessage(message2);
	} else {
		message2.setAddress("/kinect2/connected");
		message2.addStringArg("Kinect 2 not found ");
		message2.addIntArg(0);
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

	
	/*
	ofSetColor(255, 0, 255);
	ofCircle(topM, 2);
	ofCircle(bottomM, 2);
	ofCircle(leftM, 2);
	ofCircle(rightM, 2);
	*/
   

	////cout << "w/" << resize << ": " << w / resize <<  " " << " h/" << resize << ": " << h / resize << endl;

	ofSetColor(255, 0, 0);
	stringstream output;

	
	if (contours.nBlobs){
		output.clear();
		output << "rectangle violet" << endl;
		output << "x: " << contours.blobs[0].centroid.x << " y: " << contours.blobs[0].centroid.y << endl;
		output << centroidM << endl;
/*
		output << "top: " << topM  << " bottom " << bottomM << endl;
		output << "left: " << leftM << " right " << rightM << endl;
*/
		output << "angle hori: " << racket1AngleHori << endl;
		output << "angle verti: " << racket1AngleVerti << endl;
	}
	ofDrawBitmapString(output.str(), w+50, 400);

	if (contours2.nBlobs){
		output.clear();
		output << "rectangle 2 violet" << endl;
		output << "x: " << contours2.blobs[0].centroid.x << " y: " << contours2.blobs[0].centroid.y << endl;
		output << centroid2M << endl;
/*
		output << "top: " << topM  << " bottom " << bottomM << endl;
		output << "left: " << leftM << " right " << rightM << endl;
*/
		output << "angle 2 hori: " << racket2AngleHori << endl;
		output << "angle 2 verti: " << racket2AngleVerti << endl;
	}

	ofDrawBitmapString(output.str(), w+50, 400 + h / 4);

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
	/*
	if (button == OF_MOUSE_BUTTON_RIGHT){
		//calculate local mouse x,y in image
		int mx = x % w / resize;
		int my = y % h / resize;
    
		//get hue value on mouse position
		findHue = hue.getPixels()[my*(int)w/resize+mx];

		cout << "hue: " << findHue << endl;
	}
	*/
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

	case OF_KEY_F11:
		tiltAngle2++;
		if (tiltAngle2 > 30) tiltAngle2 = 30;
		kinect2.setCameraTiltAngle(tiltAngle2);
		break;

	case OF_KEY_F12:
		tiltAngle2--;
		if (tiltAngle1 < -30) tiltAngle2 = 30;
		kinect2.setCameraTiltAngle(tiltAngle2);
		break;

	case 'v':
		showVideoFeed = ! showVideoFeed;
	break;

	default:
		cout << key << " " << (char)key << endl;
		break;
	}

}



