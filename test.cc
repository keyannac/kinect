import org.openkinect.processing.*;
Kinect kinect;
void setup(){
	kinect = new Kinect(this);
	kinect.initDevice();
}

PImage img = kinect.getDepthImage();
image(img,0,0);
int[] depth = kinect.getRawDepth();


float[] depthLookup = new float[2048];

for (int i = 0; i < depthLookup.length; i++){
	depthLookup[] = rawDepthToMeters(i);
}

float rawDepthToMeters(int depthValue){
	if (depthValue < 2047){
		return (float)(1.0/((double)(depthValue)*-0.0030711016 + 3.3309495161));
	}
	return 0.0f;
}
