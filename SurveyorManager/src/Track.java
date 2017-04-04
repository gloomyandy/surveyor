import java.awt.EventQueue;

import javax.swing.JFrame;
import javax.swing.BoxLayout;
import javax.swing.JScrollPane;
import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.JSeparator;
import javax.swing.JViewport;
import javax.swing.SwingConstants;

import net.miginfocom.swing.MigLayout;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.awt.Point;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;
import javax.swing.JSlider;
import javax.swing.UIManager;
import javax.swing.event.ChangeListener;
import javax.swing.event.ChangeEvent;
import javax.swing.JButton;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

import javax.swing.ScrollPaneConstants;

import lejos.robotics.navigation.Pose;
import javax.swing.JCheckBox;


public class Track
{

    private JFrame frame;
    public JScrollPane scrollPane;
    TrackInfoView view;
    static Track window = null;
    private JTable playbackTable;
    JSlider playbackPos;
    JButton play;
    JButton run;
    
    public class TrackInfoView
    {
        private JPanel panel;
        private JTable table;
        private TrackDisplay map;
        protected boolean active;
        RobotInfo robot;

        public TrackInfoView(JPanel panel, JTable table, TrackDisplay map)
        {
            this.panel = panel;
            this.table = table;
            this.map = map;
            this.active = false;
            panel.setVisible(false);
        }

        protected void bind(RobotInfo robot)
        {
            this.robot = robot;
            update();
            panel.setVisible(true);
            active = true;
        }

        protected void release()
        {
            panel.setVisible(false);
            active = false;
            map.repaint();
        }
        
        public void update()
        {
            table.setValueAt(robot.name, 0, 1);
            table.setValueAt(String.format("(%3.1f, %3.1f)[%3.1f]", robot.gyroPose.getX(), robot.gyroPose.getY(), robot.gyroPose.getHeading()), 1, 1);
            table.setValueAt(String.format("(%3.1f, %3.1f)[%3.1f]", robot.slamPose.getX(), robot.slamPose.getY(), robot.slamPose.getHeading()), 2, 1);
            if (robot.targetPose == null)
                table.setValueAt("(-, -)[-]", 3, 1);
            else
                table.setValueAt(String.format("(%3.1f, %3.1f)[%3.1f]", robot.targetPose.getX(), robot.targetPose.getY(), robot.targetPose.getHeading()), 3, 1);
            table.setValueAt(String.format("%3.1f", robot.actualSpeed), 4, 1);
            table.setValueAt(String.format("%3.1f", robot.battery), 5, 1);
            table.setValueAt(String.format("%3.1f", robot.infraRed), 6, 1);
            table.setValueAt(String.format("%3.1f", robot.heading), 7, 1);
            playbackTable.setValueAt(""+robot.currentScanNo+"/"+(robot.processedScans-1), 0, 1);
            playbackTable.setValueAt(String.format("%3.1f", robot.distance), 1, 1);
            playbackTable.setValueAt(robot.getState(), 2, 1);
            if (!playbackPos.getValueIsAdjusting() && robot.getState() != RobotInfo.RunState.SETPOS)
                playbackPos.setValue(robot.currentScanNo);
            if (robot.processedScans > 0)
                playbackPos.setMaximum(robot.processedScans-1);
            if (robot.getState() != RobotInfo.RunState.PAUSE)
                play.setText(" || ");
            else
                play.setText(" > ");
            
            if (robot.getState() == RobotInfo.RunState.RUN)
                run.setText("Pause");
            else
                run.setText("Run");
/*            
            try
            {
                EventQueue.invokeAndWait(new Runnable()
                {
                    public void run()
                    {
                        try
                        {
                            map.paintImmediately(map.getVisibleRect());;    
                        } catch (Exception e)
                        {
                            e.printStackTrace();
                        }
                    }
                });
            } catch (InvocationTargetException | InterruptedException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
*/
            //map.repaint();
        }
        
        public TrackDisplay getMap()
        {
            return map;
        }
        
        protected void changeSpeed(int newSpeed)
        {
            robot.setSpeed(newSpeed);
        }
        
        protected void setTarget(RobotInfo.PlanState state)
        {
            Point pos = map.getTargetPosition();
            if (pos == null || state == RobotInfo.PlanState.NONE)
                robot.setTargetPose(null, RobotInfo.PlanState.NONE);
            else 
                robot.setTargetPose(new Pose((float)pos.getX(), (float)pos.getY(), (float)0.0), state);
            map.clearTargetPosition();
        }
        
    }

    /**
     * Launch the application.
     * @throws InterruptedException 
     * @throws InvocationTargetException 
     */
    public static void main(String[] args) throws InvocationTargetException, InterruptedException
    {

        EventQueue.invokeAndWait(new Runnable()
        {
            public void run()
            {
                try
                {
                    window = new Track();
                    window.frame.setVisible(true);
                    //RobotInfo.waitForRobots(window);
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        });
        RobotInfo.loadHistory(window, "/home/andy/slam/BreezySLAM-master/examples/log16.dat");
        //RobotInfo.loadHistory(window, "/home/andy/Lego/ev3/eclipsews/Surveyor/bin/log.dat");
        RobotInfo.waitForRobots(window);
    }

    /**
     * Create the application.
     */
    public Track()
    {
        initialize();
    }

    /**
     * Initialize the contents of the frame.
     */
    private void initialize()
    {
        frame = new JFrame();
        frame.setBounds(100, 100, 1500, 800);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        //frame.getContentPane().setLayout(new MigLayout("", "[500px:500px,grow,left][200px,right]", "[grow,grow]"));
        frame.getContentPane().setLayout(new MigLayout("", "[500px:500px,grow,left][240px,right]", "[grow, grow]"));
        TrackDisplay display = new TrackDisplay(RobotInfo.MAP_SIZE_PIXELS*4, RobotInfo.MAP_SIZE_METERS, RobotInfo.MAP_SIZE_PIXELS);

        scrollPane = new JScrollPane(display);
        scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
        scrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_ALWAYS);
        scrollPane.setWheelScrollingEnabled(false);
        scrollPane.getViewport().setScrollMode(JViewport.SIMPLE_SCROLL_MODE);
        //scrollPane.setDoubleBuffered(true);
        frame.getContentPane().add(scrollPane, "cell 0 0,grow");
        JPanel panel = new JPanel();
        frame.getContentPane().add(panel, "cell 1 0,alignx left,growy");
        panel.setLayout(new MigLayout("", "[200%,grow]", "[10%,grow][25%][25%][25%,grow]"));
        createGlobalControls(panel);
        createTrackInfoView(panel, display, 0);
        createPlaybackControls(panel);

        //createTrackInfoView(panel, display, 1);
        //createTrackInfoView(panel, display, 2);
        //createTrackInfoView(panel, display, 3);
    }

    private void createPlaybackControls(JPanel panel)
    {
        JPanel playback = new JPanel();
        panel.add(playback, "cell 0 3,grow");
        playback.setLayout(new MigLayout("", "[16.6%][16.6%][16.6%][16.6%][16.6%][16.6%]", "[][][][][]"));
        
        playbackTable = new JTable();
        playbackTable.setModel(new DefaultTableModel(
            new Object[][] {
                {"Scan", null},
                {"Quality", null},
                {"State", null},
                {null, null},
            },
            new String[] {
                "New column", "New column"
            }
        ) {
            Class[] columnTypes = new Class[] {
                String.class, Object.class
            };
            public Class getColumnClass(int columnIndex) {
                return columnTypes[columnIndex];
            }
        });
        playback.add(playbackTable, "span, grow, alignx left,aligny top");
        
        playbackPos = new JSlider();
        playbackPos.addChangeListener(new ChangeListener() {
            public void stateChanged(ChangeEvent arg0) {
                JSlider source = (JSlider)arg0.getSource();
                    int pos = (int)source.getValue();
                    if (!source.getValueIsAdjusting()) {
                        view.robot.setPlaybackPos(pos);
                    } else {
                        if (view.robot.currentScanNo != pos)
                            view.robot.setPlaybackPos(pos);
                    }
            }
        });
        playbackPos.setPaintTicks(true);
        playbackPos.setPaintLabels(true);
        playback.add(playbackPos, "cell 0 1 6 1,alignx center,aligny top");
        
        JButton start = new JButton("|<<");
        start.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.robot.setState(RobotInfo.RunState.START);
            }
        });
        start.setMargin(new Insets(0,0,4,4));
        playback.add(start, "cell 0 4, alignx center,aligny top");
        
        JButton rewind = new JButton("<<");
        rewind.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.robot.setState(RobotInfo.RunState.REWIND);
            }
        });
        rewind.setMargin(new Insets(0,0,4,4));
        playback.add(rewind, "cell 1 4,alignx center,aligny top");
        
        play = new JButton(" > ");
        play.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                if (view.robot.getState() != RobotInfo.RunState.PAUSE)
                {
                    view.robot.setState(RobotInfo.RunState.PAUSE);
                }
                else
                {
                    view.robot.setState(RobotInfo.RunState.PLAY);
                }
                    
            }
        });
        play.setMargin(new Insets(0,0,4,4));
        playback.add(play, "cell 2 4,alignx center,aligny top");
        
        JButton step = new JButton(">|");
        step.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.robot.setState(RobotInfo.RunState.STEP);
            }
        });
        step.setMargin(new Insets(0,0,4,4));
        playback.add(step, "cell 3 4,alignx center,aligny top");
        
        JButton forward = new JButton(">>");
        forward.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.robot.setState(RobotInfo.RunState.FORWARD);
            }
        });
        forward.setMargin(new Insets(0,0,4,4));
        playback.add(forward, "cell 4 4,alignx center,aligny top");
        
        JButton end = new JButton(">>|");
        end.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.robot.setState(RobotInfo.RunState.END);
            }
        });

        end.setMargin(new Insets(0,0,4,4));
        playback.add(end, "cell 5 4,alignx center,aligny top");
        
    }

    private void createGlobalControls(JPanel panel)
    {
        JPanel panel_1;
        panel_1 = new JPanel();
        panel.add(panel_1, "cell 0 0, grow");
        panel_1.setLayout(new MigLayout("", "[100%,grow]", "[][]"));
        JButton btnStop = new JButton("STOP");
        btnStop.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setState(RobotInfo.RunState.EXIT);
            }
        });
        panel_1.add(btnStop, "flowy,cell 0 0, grow");
        
        JCheckBox chckbxNewCheckBox1 = new JCheckBox("Show slam");
        chckbxNewCheckBox1.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(ScanInfo.POSE_SLAM, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });
        panel_1.add(chckbxNewCheckBox1, "cell 0 1");
        JCheckBox chckbxNewCheckBox2 = new JCheckBox("Show gyro");
        chckbxNewCheckBox2.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(ScanInfo.POSE_GYRO, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });
        panel_1.add(chckbxNewCheckBox2, "cell 0 2");
        JCheckBox chckbxNewCheckBox3 = new JCheckBox("Show odo");
        chckbxNewCheckBox3.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(ScanInfo.POSE_ODO, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });
        panel_1.add(chckbxNewCheckBox3, "cell 0 3");
        JCheckBox chckbxNewCheckBox4 = new JCheckBox("Show scan");
        chckbxNewCheckBox4.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(RobotInfo.SHOW_SCANS, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });

        panel_1.add(chckbxNewCheckBox4, "cell 0 4");
        JCheckBox chckbxNewCheckBox5 = new JCheckBox("Show scan history");
        chckbxNewCheckBox5.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(RobotInfo.SHOW_SCAN_HISTORY, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });
        panel_1.add(chckbxNewCheckBox5, "cell 0 5");
        JCheckBox chckbxNewCheckBox6 = new JCheckBox("Show path planning");
        chckbxNewCheckBox6.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent arg0) {
                view.robot.setDisplayOption(RobotInfo.SHOW_PATH_PLANNING, ((JCheckBox)arg0.getSource()).isSelected());
            }
        });
        panel_1.add(chckbxNewCheckBox6, "cell 0 6");
        chckbxNewCheckBox1.setSelected(true);    
        //chckbxNewCheckBox2.setSelected(view.robot.getDisplayOption(ScanInfo.POSE_GYRO));    
        //chckbxNewCheckBox3.setSelected(view.robot.getDisplayOption(ScanInfo.POSE_ODO));    
        chckbxNewCheckBox4.setSelected(true);    
        //chckbxNewCheckBox5.setSelected(view.robot.getDisplayOption(RobotInfo.SHOW_SCAN_HISTORY));    
    }
    
    private void createTrackInfoView(JPanel panel, TrackDisplay display, final int pos)
    {
        JPanel panel_1;
        JTable table_1;
        JSlider slider_2;
        panel_1 = new JPanel();
        panel.add(panel_1, "cell 0 " + (pos + 1) + ",grow");
        panel_1.setLayout(new MigLayout("", "[100%,grow]", "[grow][]"));
        
        table_1 = new JTable();
        table_1.setModel(new DefaultTableModel(
            new Object[][] {
                {"Name", null},
                {"Odo", null},
                {"Slam", null},
                {"Target", null},
                {"Speed", null},
                {"Bat", null},
                {"IR", null},
                {"Head", null},
            },
            new String[] {
                "", ""
            }
        ) {
            Class[] columnTypes = new Class[] {
                String.class, Object.class
            };
            public Class getColumnClass(int columnIndex) {
                return columnTypes[columnIndex];
            }
        });
        table_1.getColumnModel().getColumn(0).setPreferredWidth(45);
        table_1.getColumnModel().getColumn(0).setMaxWidth(45);
        panel_1.add(table_1, "flowy,cell 0 0,grow");
        
        JSlider slider_1;
        slider_1 = new JSlider();
        slider_1.setMaximum(360);
        //slider_1.setValue(10);
        slider_1.setValue(180);
        slider_1.addChangeListener(new ChangeListener() {
            public void stateChanged(ChangeEvent arg0) {
                JSlider source = (JSlider)arg0.getSource();
                if (!source.getValueIsAdjusting()) {
                    int speed = (int)source.getValue();
                    //RobotInfo.P = (float)speed/100;
                    view.robot.heading = 180.0f - (float)speed;
                }

            }
        });
        
        run = new JButton("Run");
        run.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                if (view.robot.getState() != RobotInfo.RunState.RUN)
                {
                    view.robot.setState(RobotInfo.RunState.RUN);
                }
                else
                {
                    view.robot.setState(RobotInfo.RunState.RUNPAUSE);
                }
                    
            }
        });
        panel_1.add(run, "flowy,cell 0 1,grow");
        slider_1.setMajorTickSpacing(90);
        slider_1.setPaintTicks(true);
        slider_1.setPaintLabels(true);
        panel_1.add(slider_1, "flowy,cell 0 2,grow");
        //panel_1.add(slider_1, "cell 0 2");          
        slider_2 = new JSlider();
        slider_2.setMaximum(50);
        slider_2.setValue(0);
        slider_2.addChangeListener(new ChangeListener() {
            public void stateChanged(ChangeEvent arg0) {
                JSlider source = (JSlider)arg0.getSource();
                if (!source.getValueIsAdjusting()) {
                    int speed = (int)source.getValue();
                    view.changeSpeed(speed);
                }

            }
        });
        slider_2.setMajorTickSpacing(10);
        slider_2.setPaintTicks(true);
        slider_2.setPaintLabels(true);
        panel_1.add(slider_2, "flowy,cell 0 3,grow");
        //panel_1.add(slider_2, "cell 0 3");  
        JButton manualTarget = new JButton("Set direct target");
        manualTarget.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.setTarget(RobotInfo.PlanState.DIRECT);
            }
        });
        panel_1.add(manualTarget, "flowy,cell 0 4,grow");
        JButton planTarget = new JButton("Set plan target");
        planTarget.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.setTarget(RobotInfo.PlanState.PLAN);
            }
        });
        panel_1.add(planTarget, "flowy,cell 0 5,grow");
        JButton clearTarget = new JButton("Clear target");
        clearTarget.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                view.setTarget(RobotInfo.PlanState.NONE);
            }
        });
        panel_1.add(clearTarget, "flowy,cell 0 6,grow");
        //panel_1.add(target, "cell 0 4");  
        view = new TrackInfoView(panel_1, table_1, display);

    }
    
    public TrackInfoView getView(RobotInfo robot)
    {
        return view;
    }
}
