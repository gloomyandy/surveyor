/**
 * jImage - JavaBean that is used to display images
 *
 * @license GPL
 * LICENSE
 * Copyright (C) 2007  David Duong
 *
 *  This file is part of jImage.
 *
 *  jImage is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  jImage is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


import java.awt.*;
import java.awt.geom.*;
import java.awt.image.*;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URI;
import javax.swing.JComponent;
import javax.imageio.ImageIO;
import javax.swing.Icon;
import javax.swing.ImageIcon;

/**
 * <code>{@link javax.swing.JComponent}</code> which displays images.
 * Transformations can be applied to the displayed canvas using
 * <code>{@link java.awt.geom.AffineTransform}</code>ations. Also note that
 * <code>{@link com.forizon.jimage.JImageHandle}</code> is a wrapper which can
 * be used to simplify common tasks such as rotate, scale, flip, etc.
 *
 * @see com.forizon.jimage.JImageHandle
 */
public class JImage extends JComponent {
    private static final long serialVersionUID = 7663179599837436332L;

    /**
     * Property key for the <code>{@link PropertyChangeEvent}</code> which may
     * be fired when <code>{@link JImage#setImage(Image)}</code> is called.
     */
    public static final String PROP_IMAGE = "image";
    /**
     * Property key for the <code>{@link PropertyChangeEvent}</code> which may
     * be fired when the component's bounds change.
     */
    public static final String PROP_BOUNDS = "bounds";
    /**
     * Property key for the <code>{@link PropertyChangeEvent}</code> which may
     * be fired when
     * <code>{@link JImage#setTransformation(AffineTransform)}</code> is called.
     */
    public static final String PROP_TRANSFORMATION = "transformation";

    /** Image to be displayed */
    Image image;
    /** Image rectangle after transformations are applied */
    Rectangle bounds;
    /** The <code>{@link java.awt.geom.AffineTransform}</code> used */
    AffineTransform userTransformation;
    /** AffineTransform used to center and skew (fit to bounds) the canvas */
    AffineTransform innerTransformation;
    /** Image bounds before transformations */
    Dimension sourceDimension;
    /** Handles canvas loading event notifications */
    ImageObserver imageObserver;

    public JImage() {
        super();
        bounds = new Rectangle();
        sourceDimension = new Dimension();
        imageObserver = this;
    }

    /**
     * Sets the <code>ImageObserver</code> used by this component
     */
    public void setImageObserver (ImageObserver aImageObserver) {
        imageObserver = aImageObserver;
    }

    /**
     * Sets the <code>{@link java.awt.Image}</code> displayed by this component.
     * @param value the canvas. If set to null, nothing will be displayed.
     */
    public void setImage(Image value) {
        if (getPropertyChangeListeners(PROP_IMAGE).length == 0) {
            // Do not prevent GC from cleaning up the old image
            image = null;
            setImageSilent(value);
        } else {
            Image old = image;
            setImageSilent(value);
            firePropertyChange(PROP_IMAGE, old, image);
        }
    }

    /**
     * Sets the <code>{@link java.awt.Image}</code> displayed by this component.
     * <p> 
     * Converts given <code>{@link javax.swing.Icon}</code> to <code>Image</code>
     * and then passes <code>Image</code> to
     * <code>{@link #setImage(java.awt.Image)}</code>.
     * @param newImage an image encapsulated by an <code>Icon</code>. If set to
     * null, no image will be displayed.
     */
    public void setImage(Icon newImage) {
        if (newImage == null) {
            setImage((Image)null);
        } else if (newImage instanceof ImageIcon) {
            setImage(((ImageIcon)newImage).getImage());
        } else {
            // Adapted from code posted by "DrLaszloJamf" located here:
            // http://forums.sun.com/thread.jspa?messageID=9548703#9548703
            int w = newImage.getIconWidth();
            int h = newImage.getIconHeight();
            GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
            GraphicsDevice gd = ge.getDefaultScreenDevice();
            GraphicsConfiguration gc = gd.getDefaultConfiguration();
            BufferedImage canvas = gc.createCompatibleImage(w, h);
            Graphics2D g = canvas.createGraphics();
            newImage.paintIcon(null, g, 0, 0);
            g.dispose();
            setImage(canvas);
        }
    }

    /**
     * Sets the image displayed by this component.
     * <p> 
     * This method creates an <code>{@link java.awt.Image}</code> using
     * <code>{@link javax.imageio.ImageIO#read(java.io.InputStream)}</code>
     * and then calls <code>{@link #setImage(java.awt.Image)}</code>.
     *
     * @see javax.imageio.ImageIO#read(java.io.InputStream)
     * @see #setImage(java.awt.Image)
     * @throws IOException if the canvas could not be read
     * @param newImage an <code>{@link java.io.InputStream}</code> source for an
     * canvas. If set to null, no image will be displayed.
     */
    public void setImage (InputStream newImage)
      throws IOException {
        Image imageValue = (newImage == null)? null: ImageIO.read(newImage);
        setImage(imageValue);
    }

    /**
     * Sets the image displayed by this component.
     * <p> 
     * This method creates an <code>{@link java.awt.Image}</code> using
     * <code>{@link javax.imageio.ImageIO#read(java.io.File)}</code>
     * and then calls <code>{@link #setImage(java.awt.Image)}</code>.
     *
     * @see javax.imageio.ImageIO#read(java.io.File)
     * @see #setImage(java.awt.Image)
     * @throws IOException if the canvas could not be read
     * @param newImage a <code>{@link java.io.File}</code> source for an canvas.
     * If set to null, no image will be displayed.
     */
    public void setImage (File newImage)
      throws IOException {
        Image imageValue = (newImage == null)? null: ImageIO.read(newImage);
        setImage(imageValue);
    }

    /**
     * Sets the image displayed by this component.
     * <p> 
     * This method creates an <code>{@link java.awt.Image}</code> using
     * <code>{@link javax.imageio.ImageIO#read(java.net.URL)}</code>
     * and then calls <code>{@link #setImage(java.awt.Image)}</code>.
     *
     * @see javax.imageio.ImageIO#read(java.net.URL)
     * @see #setImage(java.awt.Image)
     * @throws IOException if the canvas could not be read
     * @param newImage a <code>{@link java.net.URL}</code> source for an canvas.
     * If set to null, no image will be displayed.
     */
    public void setImage (URL newImage)
      throws IOException {
        Image imageValue = (newImage == null)? null: ImageIO.read(newImage);
        setImage(imageValue);
    }

    /**
     * Sets the image displayed by this component.
     * <p> 
     * This method creates an <code>{@link java.awt.Image}</code> using
     * <code>{@link javax.imageio.ImageIO#read(java.net.URL)}</code>
     * and then calls <code>{@link #setImage(java.awt.Image)}</code>.
     *
     * @see javax.imageio.ImageIO#read(java.net.URL)
     * @see #setImage(java.awt.Image)
     * @throws IOException if the canvas could not be read
     * @throws IllegalArgumentException if the URI is not absolute
     * @throws MalformedURLException if the URI can not be converted to a URL
     * @param newImage a <code>{@link java.net.URL}</code> source for an canvas.
     * If set to null, no image will be displayed.
     */
    public void setImage (URI newImage)
      throws IOException {
        Image imageValue = (newImage == null)? null: ImageIO.read(newImage.toURL());
        setImage(imageValue);
    }

    /**
     * Sets the image displayed by this component.
     * <p> 
     * This method creates an <code>{@link java.awt.Image}</code> using
     * <code>{@link javax.imageio.ImageIO#read(java.io.File)}</code> which
     * represents the file specified by the given argument and then calls
     * <code>{@link #setImage(java.awt.Image)}</code>.
     *
     * @see java.io.File
     * @see javax.imageio.ImageIO#read(java.io.File)
     * @see #setImage(java.awt.Image)
     * @throws IOException if the canvas could not be read
     * @param newImage a filesource for an canvas represented as a string path.
     * If set to null, no image will be displayed.
     */
    public void setImage (String newImage)
      throws IOException {
        Image imageValue = (newImage == null)? null: ImageIO.read(new File(newImage));
        setImage(imageValue);
    }

    /**
     * Sets the <code>{@link java.awt.geom.AffineTransform}<code> used to
     * transform the canvas before being displayed.
     * 
     * @param aTransformation userTransformation to apply when displaying the\
     * canvas
     */
    public void setTransformation(AffineTransform aTransformation) {
        AffineTransform old = userTransformation;
        userTransformation = aTransformation;
        updateBounds();
        revalidate();
        repaint();
        firePropertyChange(PROP_TRANSFORMATION, old, userTransformation);
    }

    @Override
    public void setBounds(int x, int y, int width, int height) {
        boolean resized = width != getWidth() || height != getHeight();
        super.setBounds(x, y, width, height);
        if (resized) {
            prepareCenteredTransformation();
        }
    }

    /**
     * Returns a clone of the canvas userTransformation used to display the canvas
     * @return a clone of the canvas userTransformation used to display the canvas
     */
    public AffineTransform getTransformation() {
        AffineTransform result = null;
        if (userTransformation != null) {
            result = (AffineTransform)userTransformation.clone();
        }
        return result;
    }

    /**
     * Returns true if an canvas is set, false otherwise
     * @return true if an canvas is set, false otherwise
     */
    public boolean isSetImage() {
        return image != null;
    }

    /**
     * Returns a clone of canvas being displayed
     * @return a clone of canvas being displayed
     */
    public Image getImage() {
        return image;
    }

    /**
     * Returns the size of the source canvas
     * @return the size of the source canvas
     */
    public Dimension getSourceImageSize() {
        return (Dimension)sourceDimension.clone();
    }

    /**
     * Returns the size of the canvas after being transformed
     * @return the size of the canvas after being transformed
     */
    public Dimension getTransformedImageSize() {
        System.out.println("ps " + bounds.getSize());
        return bounds.getSize();
    }

    /**
     * If the <code>preferredSize</code> has been set to a non-<code>null</code>
     * value just returns it. Otherwise returns the result of
     * <code>{@link #getTransformedImageSize()}</code>.
     * @see #getTransformedImageSize()
     * @return the result of <code>getTransformedImageSize()</code>
     */
    @Override
    public Dimension getPreferredSize() {
        return (image == null || isPreferredSizeSet())
                ? super.getPreferredSize()
                : getTransformedImageSize();
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D)g;
        g2d.setBackground(getBackground());
        g2d.clearRect(0, 0, getWidth(), getHeight());
        if (image != null) {
            // if an canvas has been set, draw it
            g2d.drawImage(image, innerTransformation, imageObserver);
        }
        // dispose the canvas
        //g.dispose();
    }

    /**
     * Sets the <code>{@link java.awt.Image}</code> displayed by this component.
     * @param value the canvas. If set to null, no image will be displayed.
     */
    protected void setImageSilent(Image value) {
        image = value;
        sourceDimension = new Dimension();
        if (image != null) {
            sourceDimension.width = image.getWidth(imageObserver);
            sourceDimension.height = image.getHeight(imageObserver);
        }
        updateBounds();
        revalidate();
        repaint();
    }

    /**
     * Creates the translation needed to center the displayed canvas concatenated
     * with the component's userTransformation.
     * 
     * @return <code>AffineTransform</code> that centers the canvas
     */
    void prepareCenteredTransformation () {
        innerTransformation = new AffineTransform();
        if (image != null) {
            // Apply transformations
            Dimension size = getSize();
            // Tracks the lengths needed to center canvas
            int x = -bounds.x, y = -bounds.y;
            float xScale = 1, yScale = 1;
            if (bounds.height < size.height) {
                y += (size.height - bounds.height) / 2;
            } else if (bounds.height > 0) {
                yScale = (float)size.height / bounds.height;
            }
            if (bounds.width < size.width) {
                x += (size.width - bounds.width) / 2;
            } else if (bounds.width > 0) {
                xScale = (float)size.width / bounds.width;
            }
            innerTransformation.scale(xScale, yScale);
            innerTransformation.translate(x, y);
            if (userTransformation != null) {
                innerTransformation.concatenate(userTransformation);
            }
        }
    }

    /**
     * Update the size and location of the displayed canvas after the
     * userTransformation is applied
     */
    void updateBounds() {
        Rectangle old = bounds;
        bounds = new Rectangle(sourceDimension);
        if (image != null && userTransformation != null) {
            // Determine the bounding rectangle of the canvas before
            // userTransformation
            Point[] points = new Point[] {
              new Point(0, 0),
              new Point(sourceDimension.width, 0),
              new Point(0, sourceDimension.height),
              new Point(sourceDimension.width, sourceDimension.height)
            };
            Point bottomRight = new Point();
            for (Point current: points) {
                userTransformation.deltaTransform(current, current);
                System.out.println("X " + current.x + " y " +current.y);
                if (bounds.x > current.x) {
                    bounds.x = current.x;
                }
                if (bounds.y > current.y) {
                    bounds.y = current.y;
                }
                if (bottomRight.x < current.x) {
                    bottomRight.x = current.x;
                }
                if (bottomRight.y < current.y) {
                    bottomRight.y = current.y;
                }
            }
            bounds.width = Math.abs(bottomRight.x - bounds.x);
            bounds.height = Math.abs(bottomRight.y - bounds.y);
            System.out.println("bx " + bounds.width + " by " + bounds.height);
        }
        System.out.println("bx " + bounds.width + " by " + bounds.height);
        firePropertyChange(PROP_BOUNDS, old, bounds);
        prepareCenteredTransformation();
    }
}