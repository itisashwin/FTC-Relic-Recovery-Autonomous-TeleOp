package ftc.vision;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import android.view.SurfaceView;


/**
 * Created by Shreyas on 9/30/17.
 */

public class FrameGrabber implements CameraBridgeViewBase.CvCameraViewListener2{
    public FrameGrabber(CameraBridgeViewBase cameraBridgeViewBase) {

        cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        cameraBridgeViewBase.setCvCameraViewListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return inputFrame.rgba();
    }
}
