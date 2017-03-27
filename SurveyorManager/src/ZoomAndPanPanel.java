import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;

import javax.swing.*;


public class ZoomAndPanPanel extends JPanel {
    protected final transient Image img;
    protected final Rectangle imgrect;
    private transient ZoomHandler handler;
    private transient DragScrollListener listener;
    protected double scale = 1.0;
    
    protected ZoomAndPanPanel(Image img) {
        super();
        this.img = img;
        this.imgrect = new Rectangle(img.getWidth(this), img.getHeight(this));
    }
    
    
    @Override protected void paintComponent(Graphics g) {
        //super.paintComponent(g);
        Graphics2D g2 = (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                            RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        int w = getWidth();
        int h = getHeight();
        int imageWidth = imgrect.width;
        int imageHeight = imgrect.height;
        //System.out.println("width " + w + " height " + h);
        double x = (w - scale * imageWidth)/2;
        double y = (h - scale * imageHeight)/2;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        AffineTransform at = AffineTransform.getTranslateInstance(x,y);
        at.scale(scale, scale);
        g2.drawImage(img, at, this);
    }
    

    @Override public Dimension getPreferredSize() {
        //Rectangle r = zoomTransform.createTransformedShape(imgrect).getBounds();
        //System.out.println("pref size " + r.width + " " + r.height);
        return new Dimension((int) (imgrect.width*scale), (int) (imgrect.height*scale));
    }

    @Override public void updateUI() {
        //System.out.println("update UI");
        removeMouseListener(listener);
        removeMouseMotionListener(listener);
        removeMouseWheelListener(handler);
        super.updateUI();
        listener = new DragScrollListener();
        addMouseListener(listener);
        addMouseMotionListener(listener);
        handler = new ZoomHandler();
        addMouseWheelListener(handler);
    }
    
    public void mouseClicked(Point p)
    {
        
    }

    protected class ZoomHandler extends MouseAdapter {
        private static final double ZOOM_MULTIPLICATION_FACTOR = 1.2;
        private static final int MIN_ZOOM = -10;
        private static final int MAX_ZOOM = 10;
        private static final int EXTENT = 1;
        private final BoundedRangeModel zoomRange = new DefaultBoundedRangeModel(0, EXTENT, MIN_ZOOM, MAX_ZOOM + EXTENT);
        @Override public void mouseWheelMoved(MouseWheelEvent e) {
            //System.out.println("Start wheel");
            int dir = e.getWheelRotation();
            int z = zoomRange.getValue();
            zoomRange.setValue(z + EXTENT * (dir > 0 ? -1 : 1));
            if (z != zoomRange.getValue()) {
                Component c = e.getComponent();
                Container p = SwingUtilities.getAncestorOfClass(JViewport.class, c);
                if (p instanceof JViewport) {
                    JViewport vport = (JViewport) p;
                    Rectangle ovr = vport.getViewRect();
                    double s = dir > 0 ? 1d / ZOOM_MULTIPLICATION_FACTOR : ZOOM_MULTIPLICATION_FACTOR;
                    // get current viewport loc
                    Point vp = ovr.getLocation();
                    // find centre
                    if (ovr.width > scale*imgrect.width)
                        vp.x = (int) (scale*imgrect.width/2);
                    else
                        vp.x += ovr.width/2;
                    // scale to new location
                    vp.x *= s;
                    vp.x -= ovr.width/2;
                    // repeat for y
                    if (ovr.height > scale*imgrect.height)
                        vp.y = (int) (scale*imgrect.height/2);
                    else
                        vp.y += ovr.height/2;
                    vp.y *= s;
                    vp.y -= ovr.height/2;
                    // save new scale
                    scale *= s;
                    // always centre if outside bounds
                    if (vp.x < 0) vp.x = 0;
                    if (vp.y < 0) vp.y = 0;
                    vport.setViewPosition(vp);
                    //System.out.println("after setviewport");
                    vport.revalidate();
                    //System.out.println("after setviewport2");
                    vport.setViewPosition(vp);
                    //System.out.println("after setviewport3");
                    c.repaint();
                }
            }
            //System.out.println("end wheel");
        }
    }


    class DragScrollListener extends MouseAdapter {
        private final Cursor defCursor = Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR);
        private final Cursor hndCursor = Cursor.getPredefinedCursor(Cursor.HAND_CURSOR);
        private final Point pp = new Point();
        @Override public void mouseDragged(MouseEvent e) {
            Component c = e.getComponent();
            Container p = SwingUtilities.getUnwrappedParent(c);
            if (p instanceof JViewport) {
                JViewport vport = (JViewport) p;
                Point cp = SwingUtilities.convertPoint(c, e.getPoint(), vport);
                Point vp = vport.getViewPosition();
                vp.translate(pp.x - cp.x, pp.y - cp.y);
                ((JComponent) c).scrollRectToVisible(new Rectangle(vp, vport.getSize()));
                pp.setLocation(cp);
            }
        }
        @Override public void mousePressed(MouseEvent e) {
            Component c = e.getComponent();
            c.setCursor(hndCursor);
            Container p = SwingUtilities.getUnwrappedParent(c);
            if (p instanceof JViewport) {
                JViewport vport = (JViewport) p;
                Point cp = SwingUtilities.convertPoint(c, e.getPoint(), vport);
                pp.setLocation(cp);
            }
        }
        @Override public void mouseReleased(MouseEvent e) {
            e.getComponent().setCursor(defCursor);
        }
        @Override
        public void mouseClicked(MouseEvent e)
        {
            // TODO Auto-generated method stub
            super.mouseClicked(e);
            Component c = e.getComponent();
            //c.setCursor(hndCursor);
            Container p = SwingUtilities.getUnwrappedParent(c);
            int w = c.getWidth();
            int h = c.getHeight();
            int imageWidth = imgrect.width;
            int imageHeight = imgrect.height;
            //System.out.println("width " + w + " height " + h);
            double x = (w - scale * imageWidth)/2;
            double y = (h - scale * imageHeight)/2;
            if (x < 0) x = 0;
            if (y < 0) y = 0;
            Point pt = e.getPoint();
            pt.setLocation((pt.x - x)/scale, (pt.y - y)/scale);
            System.out.println("mp " + e.getPoint() + " sp " + pt );
            ZoomAndPanPanel.this.mouseClicked(pt);
        }
        
        
    }
}