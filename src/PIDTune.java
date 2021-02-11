import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Random;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class PIDTune extends JPanel{
    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    Random rand= new Random();
    float error = 0.0f;
    float last_error = 0.0f;
    float error2 = 0.0f;
    float last_error2 = 0.0f;
    float output = 0.0f;
    float output2 = 0.0f;
    float thrust = 9.81f;
    float cx;
    float cy;
    float cz;
    float px;
    float py;
    float pz;
    float cvx;
    float cvy;
    float cvz;
    float pvx;
    float pvy;
    float pvz;
    JButton go = new JButton("go");
    JFrame jfrm = new JFrame("PID tune");
    int width = 1900;
    int height = 1000;
    boolean start = false;
    static PIDTune pt;
    float angle1;
    float angle2;
    class PIDThread extends Thread{
        float delta_time = (float)(1000*1.0/60.0);
        float Kp = 1.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
        float chromasome[][] = new float[100][4];
        @Override
        public void run() {
            init();
            rand = new Random();  
            for (int j = 0; j < 100; j++) {
                chromasome[j][0] = 10.0f * (rand.nextFloat() - 0.5f);
                chromasome[j][1] = 10.0f * (rand.nextFloat() - 0.5f);
                chromasome[j][2] = 10.0f * (rand.nextFloat() - 0.5f);
                chromasome[j][3] = 1000000.0f;
            }
            float closest = 1000000f;
            for(int iteration = 0; iteration < 1000 && closest > 0.1f; iteration++) {
                int i = 0;
                while(i < 100) {
                    Kp = chromasome[i][0];
                    Ki = chromasome[i][1];
                    Kd = chromasome[i][2];
                    rand = new Random(0);  
                    float sumDist = 0.0f;
                    for (int k = 0; k < 20 ; k++) {
                        initTestScene();
                        closest = 1000000f;
                        float accumulation_of_error = 0.0f;
                        float derivative_of_error = 0.0f;
                        float accumulation_of_error2 = 0.0f;
                        float derivative_of_error2 = 0.0f;
              
                        int l = 0;
                        while(l++ < 1200) {
                            //Date currentTime = new Date();
                            float distToTarget = testScene();
                            accumulation_of_error += error * delta_time;
                            derivative_of_error = (error - last_error) / delta_time;
                            last_error = error;
                            output = (error * Kp) + (accumulation_of_error * Ki) + (derivative_of_error * Kd);
                            accumulation_of_error2 += error2 * delta_time;
                            derivative_of_error2 = (error2 - last_error2) / delta_time;
                            last_error2 = error2;
                            output2 = (error2 * Kp) + (accumulation_of_error2 * Ki) + (derivative_of_error2 * Kd);
                            updateTestScene();
                            pt.repaint();

                            if (distToTarget < closest) {
                                closest = distToTarget;
                            }
//                                                    try {
//                                                        Thread.sleep((long)(2));
//                                                    } catch (InterruptedException e) {
//                                                        // TODO Auto-generated catch block
//                                                        e.printStackTrace();
//                                                    }
                            //Date afterTime = new Date();
                            delta_time = 1.0f / 60.0f;//(float)((afterTime.getTime() - currentTime.getTime())/1000.0f);
                        }
                        sumDist += closest;
                        
                    }
//                    while(!start) {
//                        try {
//                         Thread.sleep(10);
//                     } catch (InterruptedException e) {
//                         // TODO Auto-generated catch block
//                         e.printStackTrace();
//                     } 
//                     }
//                    start = !start;
                    chromasome[i][3] = sumDist;
                    i++;
                    init();
                }
                int closestJ = -1;
                float distClosest = 1000000f;
                int secClosestJ = -1;
                float secDistClosest = 1000000f;
                for (int j = 0; j < 100; j++) {
                    if (chromasome[j][3] < distClosest) {
                        secDistClosest = distClosest;
                        secClosestJ = closestJ;
                        closestJ = j;
                        distClosest = chromasome[j][3];
                    }
                }
                System.out.println(""+iteration+", "+chromasome[closestJ][0]+", "+chromasome[closestJ][1]+", "+chromasome[closestJ][2]+", "+chromasome[closestJ][3]);

                if(secClosestJ == -1) {
                    secClosestJ = closestJ;
                    secDistClosest = distClosest;
                }
                // breed/mutate
                for (int j = 0; j < 100; j++) {

                    float result[] = new float[4];
                    result[0] = rand.nextFloat() >0.5f?chromasome[closestJ][0]:chromasome[secClosestJ][0];
                    result[1] = rand.nextFloat() >0.5f?chromasome[closestJ][1]:chromasome[secClosestJ][1];
                    result[2] = rand.nextFloat() >0.5f?chromasome[closestJ][2]:chromasome[secClosestJ][2];
                    result[3] = chromasome[closestJ][3];
                    chromasome[j][0] = result[0]+(10.0f / (iteration+1)) * (rand.nextFloat() - 0.5f);
                    chromasome[j][1] = result[1]+(10.0f / (iteration+1)) * (rand.nextFloat() - 0.5f);
                    chromasome[j][2] = result[2]+(10.0f / (iteration+1)) * (rand.nextFloat() - 0.5f);
                    closest = 1000000f;
                    chromasome[j][3] = 1000000f;
                }

            }
        }
        private void initTestScene() {
            cx = (rand.nextFloat()-0.5f) * 100.0f;
            cy = (rand.nextFloat()-0.5f) * 100.0f;
            cz = (rand.nextFloat()-0.5f) * 100.0f;
            px = (rand.nextFloat()-0.5f) * 100.0f;
            py = (rand.nextFloat()-0.5f) * 100.0f;
            pz = (rand.nextFloat()-0.5f) * 100.0f;
            cvx = (rand.nextFloat()-0.5f) * 10.0f;
            cvy = (rand.nextFloat()-0.5f) * 10.0f;
            cvz = (rand.nextFloat()-0.5f) * 10.0f;
            pvx = (rand.nextFloat()-0.5f) * 10.0f;
            pvy = (rand.nextFloat()-0.5f) * 10.0f;
            pvz = (rand.nextFloat()-0.5f) * 10.0f;          
        }
        private float testScene() {
            cx += cvx;
            cy += cvy;
            cz += cvz;
            angle1 = (float)((180.0 / Math.PI)*Math.acos((cz-pz)/Math.sqrt((cx-px)*(cx-px)+(cy-py)*(cy-py)+(cz-pz)*(cz-pz))));
            angle2 = (float)((180.0 / Math.PI)*Math.atan2((cy-py),(cx-px)));
            float oldAngle1 = (float)((180.0 / Math.PI)*Math.acos((pvz)/Math.sqrt((pvx)*(pvx)+(pvy)*(pvy)+(pvz)*(pvz))));
            float oldAngle2 = (float)((180.0 / Math.PI)*Math.atan2((pvy),(pvx)));
            error = angle1 - oldAngle1;
            error2= angle2 - oldAngle2;

            return (float)Math.sqrt((cx-px)*(cx-px)+(cy-py)*(cy-py)+(cz-pz)*(cz-pz));

        }
        private void updateTestScene() {
            pvx = (float) (Math.sin(Math.PI*output/180.0)*Math.cos(Math.PI*output2/180.0));
            pvy = (float) (Math.sin(Math.PI*output/180.0)*Math.sin(Math.PI*output2/180.0));
            pvz = (float) Math.cos(Math.PI*output/180.0);
            float dcpx = cx -px;
            float dcpy = cy -py;
            float dcpz = cz -pz;
            float mag = (float) Math.sqrt(dcpx*dcpx+dcpy*dcpy+dcpz*dcpz);
            dcpx = dcpx/mag;
            dcpy = dcpy/mag;
            dcpz = dcpz/mag;
            px += (dcpx + pvx) * thrust /60.0f;
            py += (dcpy + pvy) * thrust /60.0f;
            pz += (dcpz + pvz) * thrust /60.0f;
        }
 
    }
    List<Integer> xlistc;
    List<Integer> ylistc;
    List<Integer> xlistp;
    List<Integer> ylistp;
    Date startTime;
    public void init() {
        xlistc = new ArrayList<Integer>();
        ylistc = new ArrayList<Integer>();
        xlistp = new ArrayList<Integer>();
        ylistp = new ArrayList<Integer>();
        startTime = new Date();
    }
    @Override
    public void paint(Graphics g) {
        super.paint(g);
        int height = getHeight();
        int width = getWidth();
        g.setColor(Color.BLACK);
        g.fillRect(0, 0, width, height);
        xlistc.add((int)((new Date()).getTime()-startTime.getTime()));//(width / 2.0 + angle1/10));
        ylistc.add((int)(height / 2.0 -angle2/10));
        xlistp.add((int)((new Date()).getTime()-startTime.getTime()));//(width / 2.0 + output/10));
        ylistp.add((int)(height / 2.0 -output2/10));
        for(int i = 0; i < xlistc.size();i++) {
        g.setColor(Color.RED);
        g.fillRect( xlistp.get(i), 
                ylistp.get(i),
                (int)(5), 
                (int)(5));
        g.setColor(Color.GREEN);
        g.fillRect(xlistc.get(i), 
                ylistc.get(i),
                (int)(5), 
                (int)(5));
        }
    }
    class PaintDemo{

        PaintDemo(){
            jfrm.setSize(width, height);
            jfrm.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


            jfrm.setLayout(new BorderLayout());

            JPanel temp = new JPanel();
            go.addActionListener(new ActionListener() {

                public void actionPerformed(ActionEvent e) {
                    
                        start = !start;   
                    
                 }

            });
            temp.add(go);

            jfrm.add(temp, BorderLayout.NORTH);
            jfrm.add(pt, BorderLayout.CENTER);

            jfrm.setVisible(true);
        }
    }

    public static void main(String[] args) {
        pt = new PIDTune();
        PIDThread thread = pt.new PIDThread();
        pt.new PaintDemo();
        thread.start();

    }

}
