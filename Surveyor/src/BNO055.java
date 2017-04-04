
import lejos.hardware.device.UART;
import lejos.hardware.port.Port;
import lejos.utility.Delay;

/**
 * A Java port of the arduino library for the BNO055 from adafruit.
 *
 * @author james@team2168.org
 *
 *
 *This is a library for the BNO055 orientation sensor
 *
 *Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *Pick one up today in the adafruit shop!
 *------> http://www.adafruit.com/products
 *
 *These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *Adafruit invests time and resources providing this open source code,
 *please support Adafruit andopen-source hardware by purchasing products
 *from Adafruit!
 *
 *Written by KTOWN for Adafruit Industries.
 *
 *MIT license, all text above must be included in any redistribution
 *
 */
public class BNO055 {
    public static final byte BNO055_ADDRESS_A = 0x28;
    public static final byte BNO055_ADDRESS_B = 0x29;
    public static final int BNO055_ID = 0xA0;

    private static final int UART_IO_MAX_LEN = 128;
    private static int _mode;
    private UART uart;
    private byte[] uartByteBuf = new byte[1];
    private byte[] uartIOBuf = new byte[UART_IO_MAX_LEN];

    public class SystemStatus {
        public int system_status;
        public int self_test_result;
        public int system_error;
    }

    public enum reg_t {
        /* Page id register definition */
        BNO055_PAGE_ID_ADDR                                     (0X07),

        /* PAGE0 REGISTER DEFINITION START*/
        BNO055_CHIP_ID_ADDR                                     (0x00),
        BNO055_ACCEL_REV_ID_ADDR                                (0x01),
        BNO055_MAG_REV_ID_ADDR                                  (0x02),
        BNO055_GYRO_REV_ID_ADDR                                 (0x03),
        BNO055_SW_REV_ID_LSB_ADDR                               (0x04),
        BNO055_SW_REV_ID_MSB_ADDR                               (0x05),
        BNO055_BL_REV_ID_ADDR                                   (0X06),

        /* Accel data register */
        BNO055_ACCEL_DATA_X_LSB_ADDR                            (0X08),
        BNO055_ACCEL_DATA_X_MSB_ADDR                            (0X09),
        BNO055_ACCEL_DATA_Y_LSB_ADDR                            (0X0A),
        BNO055_ACCEL_DATA_Y_MSB_ADDR                            (0X0B),
        BNO055_ACCEL_DATA_Z_LSB_ADDR                            (0X0C),
        BNO055_ACCEL_DATA_Z_MSB_ADDR                            (0X0D),

        /* Mag data register */
        BNO055_MAG_DATA_X_LSB_ADDR                              (0X0E),
        BNO055_MAG_DATA_X_MSB_ADDR                              (0X0F),
        BNO055_MAG_DATA_Y_LSB_ADDR                              (0X10),
        BNO055_MAG_DATA_Y_MSB_ADDR                              (0X11),
        BNO055_MAG_DATA_Z_LSB_ADDR                              (0X12),
        BNO055_MAG_DATA_Z_MSB_ADDR                              (0X13),

        /* Gyro data registers */
        BNO055_GYRO_DATA_X_LSB_ADDR                             (0X14),
        BNO055_GYRO_DATA_X_MSB_ADDR                             (0X15),
        BNO055_GYRO_DATA_Y_LSB_ADDR                             (0X16),
        BNO055_GYRO_DATA_Y_MSB_ADDR                             (0X17),
        BNO055_GYRO_DATA_Z_LSB_ADDR                             (0X18),
        BNO055_GYRO_DATA_Z_MSB_ADDR                             (0X19),

        /* Euler data registers */
        BNO055_EULER_H_LSB_ADDR                                 (0X1A),
        BNO055_EULER_H_MSB_ADDR                                 (0X1B),
        BNO055_EULER_R_LSB_ADDR                                 (0X1C),
        BNO055_EULER_R_MSB_ADDR                                 (0X1D),
        BNO055_EULER_P_LSB_ADDR                                 (0X1E),
        BNO055_EULER_P_MSB_ADDR                                 (0X1F),

        /* Quaternion data registers */
        BNO055_QUATERNION_DATA_W_LSB_ADDR                       (0X20),
        BNO055_QUATERNION_DATA_W_MSB_ADDR                       (0X21),
        BNO055_QUATERNION_DATA_X_LSB_ADDR                       (0X22),
        BNO055_QUATERNION_DATA_X_MSB_ADDR                       (0X23),
        BNO055_QUATERNION_DATA_Y_LSB_ADDR                       (0X24),
        BNO055_QUATERNION_DATA_Y_MSB_ADDR                       (0X25),
        BNO055_QUATERNION_DATA_Z_LSB_ADDR                       (0X26),
        BNO055_QUATERNION_DATA_Z_MSB_ADDR                       (0X27),

        /* Linear acceleration data registers */
        BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     (0X28),
        BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     (0X29),
        BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     (0X2A),
        BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     (0X2B),
        BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     (0X2C),
        BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     (0X2D),

        /* Gravity data registers */
        BNO055_GRAVITY_DATA_X_LSB_ADDR                          (0X2E),
        BNO055_GRAVITY_DATA_X_MSB_ADDR                          (0X2F),
        BNO055_GRAVITY_DATA_Y_LSB_ADDR                          (0X30),
        BNO055_GRAVITY_DATA_Y_MSB_ADDR                          (0X31),
        BNO055_GRAVITY_DATA_Z_LSB_ADDR                          (0X32),
        BNO055_GRAVITY_DATA_Z_MSB_ADDR                          (0X33),

        /* Temperature data register */
        BNO055_TEMP_ADDR                                        (0X34),

        /* Status registers */
        BNO055_CALIB_STAT_ADDR                                  (0X35),
        BNO055_SELFTEST_RESULT_ADDR                             (0X36),
        BNO055_INTR_STAT_ADDR                                   (0X37),

        BNO055_SYS_CLK_STAT_ADDR                                (0X38),
        BNO055_SYS_STAT_ADDR                                    (0X39),
        BNO055_SYS_ERR_ADDR                                     (0X3A),

        /* Unit selection register */
        BNO055_UNIT_SEL_ADDR                                    (0X3B),
        BNO055_DATA_SELECT_ADDR                                 (0X3C),

        /* Mode registers */
        BNO055_OPR_MODE_ADDR                                    (0X3D),
        BNO055_PWR_MODE_ADDR                                    (0X3E),

        BNO055_SYS_TRIGGER_ADDR                                 (0X3F),
        BNO055_TEMP_SOURCE_ADDR                                 (0X40),

        /* Axis remap registers */
        BNO055_AXIS_MAP_CONFIG_ADDR                             (0X41),
        BNO055_AXIS_MAP_SIGN_ADDR                               (0X42),

        /* SIC registers */
        BNO055_SIC_MATRIX_0_LSB_ADDR                            (0X43),
        BNO055_SIC_MATRIX_0_MSB_ADDR                            (0X44),
        BNO055_SIC_MATRIX_1_LSB_ADDR                            (0X45),
        BNO055_SIC_MATRIX_1_MSB_ADDR                            (0X46),
        BNO055_SIC_MATRIX_2_LSB_ADDR                            (0X47),
        BNO055_SIC_MATRIX_2_MSB_ADDR                            (0X48),
        BNO055_SIC_MATRIX_3_LSB_ADDR                            (0X49),
        BNO055_SIC_MATRIX_3_MSB_ADDR                            (0X4A),
        BNO055_SIC_MATRIX_4_LSB_ADDR                            (0X4B),
        BNO055_SIC_MATRIX_4_MSB_ADDR                            (0X4C),
        BNO055_SIC_MATRIX_5_LSB_ADDR                            (0X4D),
        BNO055_SIC_MATRIX_5_MSB_ADDR                            (0X4E),
        BNO055_SIC_MATRIX_6_LSB_ADDR                            (0X4F),
        BNO055_SIC_MATRIX_6_MSB_ADDR                            (0X50),
        BNO055_SIC_MATRIX_7_LSB_ADDR                            (0X51),
        BNO055_SIC_MATRIX_7_MSB_ADDR                            (0X52),
        BNO055_SIC_MATRIX_8_LSB_ADDR                            (0X53),
        BNO055_SIC_MATRIX_8_MSB_ADDR                            (0X54),

        /* Accelerometer Offset registers */
        ACCEL_OFFSET_X_LSB_ADDR                                 (0X55),
        ACCEL_OFFSET_X_MSB_ADDR                                 (0X56),
        ACCEL_OFFSET_Y_LSB_ADDR                                 (0X57),
        ACCEL_OFFSET_Y_MSB_ADDR                                 (0X58),
        ACCEL_OFFSET_Z_LSB_ADDR                                 (0X59),
        ACCEL_OFFSET_Z_MSB_ADDR                                 (0X5A),

        /* Magnetometer Offset registers */
        MAG_OFFSET_X_LSB_ADDR                                   (0X5B),
        MAG_OFFSET_X_MSB_ADDR                                   (0X5C),
        MAG_OFFSET_Y_LSB_ADDR                                   (0X5D),
        MAG_OFFSET_Y_MSB_ADDR                                   (0X5E),
        MAG_OFFSET_Z_LSB_ADDR                                   (0X5F),
        MAG_OFFSET_Z_MSB_ADDR                                   (0X60),

        /* Gyroscope Offset register s*/
        GYRO_OFFSET_X_LSB_ADDR                                  (0X61),
        GYRO_OFFSET_X_MSB_ADDR                                  (0X62),
        GYRO_OFFSET_Y_LSB_ADDR                                  (0X63),
        GYRO_OFFSET_Y_MSB_ADDR                                  (0X64),
        GYRO_OFFSET_Z_LSB_ADDR                                  (0X65),
        GYRO_OFFSET_Z_MSB_ADDR                                  (0X66),

        /* Radius registers */
        ACCEL_RADIUS_LSB_ADDR                                   (0X67),
        ACCEL_RADIUS_MSB_ADDR                                   (0X68),
        MAG_RADIUS_LSB_ADDR                                     (0X69),
        MAG_RADIUS_MSB_ADDR                                     (0X6A);

        private final int val;

        reg_t(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    };

    public enum powermode_t {
        POWER_MODE_NORMAL                                       (0X00),
        POWER_MODE_LOWPOWER                                     (0X01),
        POWER_MODE_SUSPEND                                      (0X02);

        private final int val;

        powermode_t(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    };

    public enum opmode_t {
        /* Operation mode settings*/
        OPERATION_MODE_CONFIG                                   (0X00),
        OPERATION_MODE_ACCONLY                                  (0X01),
        OPERATION_MODE_MAGONLY                                  (0X02),
        OPERATION_MODE_GYRONLY                                  (0X03),
        OPERATION_MODE_ACCMAG                                   (0X04),
        OPERATION_MODE_ACCGYRO                                  (0X05),
        OPERATION_MODE_MAGGYRO                                  (0X06),
        OPERATION_MODE_AMG                                      (0X07),
        OPERATION_MODE_IMUPLUS                                  (0X08),
        OPERATION_MODE_COMPASS                                  (0X09),
        OPERATION_MODE_M4G                                      (0X0A),
        OPERATION_MODE_NDOF_FMC_OFF                             (0X0B),
        OPERATION_MODE_NDOF                                     (0X0C);

        private final int val;

        opmode_t(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    }

    public class RevInfo {
        public byte accel_rev;
        public byte mag_rev;
        public byte gyro_rev;
        public short sw_rev;
        public byte bl_rev;
    }

    public class CalData {
        public byte sys;
        public byte gyro;
        public byte accel;
        public byte mag;
    }

    public enum vector_type_t {
        VECTOR_ACCELEROMETER (reg_t.BNO055_ACCEL_DATA_X_LSB_ADDR.getVal()),
        VECTOR_MAGNETOMETER  (reg_t.BNO055_MAG_DATA_X_LSB_ADDR.getVal()),
        VECTOR_GYROSCOPE     (reg_t.BNO055_GYRO_DATA_X_LSB_ADDR.getVal()),
        VECTOR_EULER         (reg_t.BNO055_EULER_H_LSB_ADDR.getVal()),
        VECTOR_LINEARACCEL   (reg_t.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR.getVal()),
        VECTOR_GRAVITY       (reg_t.BNO055_GRAVITY_DATA_X_LSB_ADDR.getVal());

        private final int val;

        vector_type_t(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    };

    private static final boolean DEBUG = false; //set true to print diagnostics messages

    /**
     * Instantiates a new BNO055 class
     *
     * @param port the physical port the sensor is plugged into on the roboRio
     * @param address the address the sensor is at (0x28 or 0x29)
     */
    public BNO055(Port port) {
        uart = new UART(port);
        uart.setBitRate(115200);
        byte[] tmp = new byte[0];
        uart.write(tmp,  0,  0);
    }



    /**
     * Sets up the HW
     *
     * @param mode
     * @return
     */
    public boolean begin(opmode_t mode) {
        /* Make sure we have the right device */
        /*
        for(int i = 0; i < 1000; i++)
        {
            read8(reg_t.BNO055_CHIP_ID_ADDR);   
            Delay.msDelay(1000);
        }
        */
        byte id = read8(reg_t.BNO055_CHIP_ID_ADDR);

        if((0xFF & id) != BNO055_ID) {
            Delay.msDelay(1000); // hold on for boot
            id = read8(reg_t.BNO055_CHIP_ID_ADDR);
            if((0xFF & id) != BNO055_ID) {
                return false;  // still not? ok bail
            }
        }

        /* Switch to config mode (just in case since this is the default) */
        setMode(opmode_t.OPERATION_MODE_CONFIG.getVal());

        /* Reset */
            write8(reg_t.BNO055_SYS_TRIGGER_ADDR, (byte) 0x20);
            while ((0xFF & read8(reg_t.BNO055_CHIP_ID_ADDR)) != BNO055_ID) {
                Delay.msDelay(10);
            }
            Delay.msDelay(50);
System.out.println("after reset");
        /* Set to normal power mode */
        write8(reg_t.BNO055_PWR_MODE_ADDR, (byte) powermode_t.POWER_MODE_NORMAL.getVal());
        Delay.msDelay(10);

        write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);

        /* Set the output units */
        /*
        byte unitsel = (0 << 7) | // Orientation = Android
                          (0 << 4) | // Temperature = Celsius
                          (0 << 2) | // Euler = Degrees
                          (1 << 1) | // Gyro = Rads
                          (0 << 0);  // Accelerometer = m/s^2
        write8(BNO055_UNIT_SEL_ADDR, unitsel);
         */

        write8(reg_t.BNO055_SYS_TRIGGER_ADDR, (byte) 0x00);
        Delay.msDelay(10);
        /* Set the requested operating mode (see section 3.3) */
        //System.out.println("about to set mode");
        //setMode(mode);

        Delay.msDelay(20);
        return true;
    }

    /**
     * Puts the chip in the specified operating mode
     * @param mode
     */
    public void setMode(opmode_t mode) {
        setMode(mode.getVal());
    }

    private void setMode(int mode){
        _mode = mode;
        while(!write8(reg_t.BNO055_OPR_MODE_ADDR, (byte) _mode))
            Delay.msDelay(100);
        Delay.msDelay(30);
    }

    /**
     * Use the external 32.768KHz crystal
     * @param usextal
     */
    public void setExtCrystalUse(boolean usextal) {
        int modeback = _mode;
System.out.println("Set xtal use");
        /* Switch to config mode (just in case since this is the default) */
System.out.println("Set mode 1");
        setMode(opmode_t.OPERATION_MODE_CONFIG.getVal());
        Delay.msDelay(25);
        write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);
        if (usextal) {
            write8(reg_t.BNO055_SYS_TRIGGER_ADDR, (byte) 0x80);
        } else {
            write8(reg_t.BNO055_SYS_TRIGGER_ADDR, (byte) 0x00);
        }
        Delay.msDelay(2000);
        /* Set the requested operating mode (see section 3.3) */
        System.out.println("Set mode 2");
        setMode(modeback);
        Delay.msDelay(20);
    }

    /**
     * Read the current calibration data and return it as an array of bytes
     * @return calibration data
     */
    public byte[] getCalibrationData()
    {
        int modeback = _mode;
        byte[] data = new byte[22];
        setMode(opmode_t.OPERATION_MODE_CONFIG.getVal());
        Delay.msDelay(25);
        readLen(reg_t.ACCEL_OFFSET_X_LSB_ADDR, data);
        setMode(modeback);
        Delay.msDelay(20);        
        return data;
    }

    /**
     * Write calibration data to the device
     * @param data calibration values
     */
    public void setCalibrationData(byte[] data)
    {
        if (data.length != 22)
            throw new IllegalArgumentException("Calibration data is not correct length");        
        int modeback = _mode;
        System.out.println("Set calibration");
        setMode(opmode_t.OPERATION_MODE_CONFIG.getVal());
        write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);
        Delay.msDelay(2000);
        while (!writeLen(reg_t.ACCEL_OFFSET_X_LSB_ADDR, data))
            Delay.msDelay(1000);
        setMode(modeback);
        Delay.msDelay(20);        

    }

    /**
     * Gets the latest system status info
     * @return
     */
    public SystemStatus getSystemStatus() {
        SystemStatus status = new SystemStatus();

        write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);

        /* System Status (see section 4.3.58)
           ---------------------------------
           0 = Idle
           1 = System Error
           2 = Initializing Peripherals
           3 = System Initalization
           4 = Executing Self-Test
           5 = Sensor fusion algorithm running
           6 = System running without fusion algorithms */

        status.system_status = read8(reg_t.BNO055_SYS_STAT_ADDR);

        /* Self Test Results (see section )
           --------------------------------
           1 = test passed, 0 = test failed
           Bit 0 = Accelerometer self test
           Bit 1 = Magnetometer self test
           Bit 2 = Gyroscope self test
           Bit 3 = MCU self test
           0x0F = all good! */

        status.self_test_result = read8(reg_t.BNO055_SELFTEST_RESULT_ADDR);

        /* System Error (see section 4.3.59)
           ---------------------------------
           0 = No error
           1 = Peripheral initialization error
           2 = System initialization error
           3 = Self test result failed
           4 = Register map value out of range
           5 = Register map address out of range
           6 = Register map write error
           7 = BNO low power mode not available for selected operation mode
           8 = Accelerometer power mode not available
           9 = Fusion algorithm configuration error
           A = Sensor configuration error */
        status.system_error = read8(reg_t.BNO055_SYS_ERR_ADDR);

        //XXX look into why this sleep is here
        Delay.msDelay(200);
        return status;
    }

    /**
     * Gets the chip revision numbers
     *
     * @return
     */
    public RevInfo getRevInfo() {
        int a = 0, b = 0;
        RevInfo info = new RevInfo();

        /* Check the accelerometer revision */
        info.accel_rev = read8(reg_t.BNO055_ACCEL_REV_ID_ADDR);

        /* Check the magnetometer revision */
        info.mag_rev   = read8(reg_t.BNO055_MAG_REV_ID_ADDR);

        /* Check the gyroscope revision */
        info.gyro_rev  = read8(reg_t.BNO055_GYRO_REV_ID_ADDR);

        /* Check the SW revision */
        info.bl_rev    = read8(reg_t.BNO055_BL_REV_ID_ADDR);

        a = read8(reg_t.BNO055_SW_REV_ID_LSB_ADDR);
        b = read8(reg_t.BNO055_SW_REV_ID_MSB_ADDR);
        info.sw_rev = (short) ((b << 8) | a);

        return info;
    }

    /**
     * Gets current calibration state.
     * @return each value will be set to 0 if not calibrated, 3 if fully calibrated.
     */
    public CalData getCalibration() {
        CalData data = new CalData();
        int rawCalData = read8(reg_t.BNO055_CALIB_STAT_ADDR);

        data.sys = (byte) ((rawCalData >> 6) & 0x03);
        data.gyro = (byte) ((rawCalData >> 4) & 0x03);
        data.accel = (byte) ((rawCalData >> 2) & 0x03);
        data.mag = (byte) (rawCalData & 0x03);

        return data;
    }

    /**
     *
     * @return temperature in degrees celsius.
     */
    public int getTemp() {

        return (read8(reg_t.BNO055_TEMP_ADDR));
    }

    /**
     * Gets a vector reading from the specified source
     *
     * @param vector_type
     * @return an array of vectors. [x,y,z]
     */
    public double[] getVector(vector_type_t vector_type) {
        final int LEN = 6;
        double[] xyz = new double[3];
        byte[] buffer = new byte[LEN];


        int x = 0, y = 0, z = 0;

        /* Read vector data (6 bytes) */
        
        readLen(vector_type.getVal(), buffer);

        //x = (((int)buffer[LEN-6]) & 0xFF) | ((((int)buffer[LEN-5]) << 8) & 0xFF00);
        //y = (((int)buffer[LEN-4]) & 0xFF) | ((((int)buffer[LEN-3]) << 8) & 0xFF00);
        //z = (((int)buffer[LEN-2]) & 0xFF) | ((((int)buffer[LEN-1]) << 8) & 0xFF00);
        x = (((int)buffer[LEN-6]) & 0xFF) | ((((int)buffer[LEN-5]) << 8));
        y = (((int)buffer[LEN-4]) & 0xFF) | ((((int)buffer[LEN-3]) << 8));
        z = (((int)buffer[LEN-2]) & 0xFF) | ((((int)buffer[LEN-1]) << 8));
        /* Convert the value to an appropriate range (section 3.6.4) */
        /* and assign the value to the Vector type */
        switch(vector_type) {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            xyz[0] = ((double)x)/16.0;
            xyz[1] = ((double)y)/16.0;
            xyz[2] = ((double)z)/16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1rps = 900 LSB */
            xyz[0] = ((double)x)/900.0;
            xyz[1] = ((double)y)/900.0;
            xyz[2] = ((double)z)/900.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            xyz[0] = ((double)x)/16.0;
            xyz[1] = ((double)y)/16.0;
            xyz[2] = ((double)z)/16.0;
            break;
        case VECTOR_ACCELEROMETER:
        case VECTOR_LINEARACCEL:
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x)/100.0;
            xyz[1] = ((double)y)/100.0;
            xyz[2] = ((double)z)/100.0;
            break;
        }

        return xyz;
    }

    //  /**************************************************************************/
    //  /*!
    //  @brief  Gets a quaternion reading from the specified source
    //   */
    //  /**************************************************************************/
    //  public imu::Quaternion Adafruit_BNO055::getQuat(void)
    //  {
    //      byte buffer[8];
    //      memset (buffer, 0, 8);
    //
    //      int16_t x, y, z, w;
    //      x = y = z = w = 0;
    //
    //      /* Read quat data (8 bytes) */
    //      readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    //      w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    //      x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    //      y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    //      z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
    //
    //      /* Assign to Quaternion */
    //      /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
    //   3.6.5.5 Orientation (Quaternion)  */
    //      const double scale = (1.0 / (1<<14));
    //      imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
    //      return quat;
    //  }

    //  /**
    //   * Reads the sensor and returns the data as a sensors_event_t
    //   */
    //  public boolean getEvent(sensors_event_t *event) {
    //      /* Clear the event */
    //      memset(event, 0, sizeof(sensors_event_t));
    //
    //      event->version   = sizeof(sensors_event_t);
    //      event->sensor_id = _sensorID;
    //      event->type      = SENSOR_TYPE_ORIENTATION;
    //      event->timestamp = millis();
    //
    //      /* Get a Euler angle sample for orientation */
    //      imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
    //      event->orientation.x = euler.x();
    //      event->orientation.y = euler.y();
    //      event->orientation.z = euler.z();
    //
    //      return true;
    //  }

    /**
     * Discard any input in the I/O buffer
     */
    private void flushInput()
    {
        while (uart.read(uartIOBuf, 0, uartIOBuf.length) > 0)
            ;        
    }
    
    
    /**
     * Writes an 8 bit value to the device
     * @param reg the register to write the data to
     * @param value a byte of data to write
     * @return True if operation succeeds 
     */
    private synchronized boolean write8(reg_t reg, byte value) {
        uartByteBuf[0] = value;
        return writeLen(reg, uartByteBuf);
    }
    

    /**
     * Writes an series of 8 bit values to the device
     * @param reg the register to write the data to
     * @param value a byte of data to write
     * @return True if operation succeeds 
     */
    private synchronized boolean writeLen(reg_t reg, byte[] data) {
        byte[] uartData = uartIOBuf;
        flushInput();
        uartData[0] = (byte)0xAA;
        uartData[1] = (byte)0;
        uartData[2] = (byte)reg.getVal();
        uartData[3] = (byte) data.length;
        if (uart.write(uartData, 0, 4) != 4)
            System.out.println("Failed to write all data");
        if (uart.write(data, 0, data.length) != data.length)
            System.out.println("Failed to write all data");
        if (!uartRead(uartData, 0, 2))
        {
            System.out.println("timeout on write to " + Integer.toHexString(reg.getVal()));
            return false;
        }
        if (uartData[0] != (byte)0xEE || uartData[1] != 1)
        {
            System.out.println("Got error on write to " + Integer.toHexString(reg.getVal()) + " value is 0x" + Integer.toHexString(uartData[0]) + 
                    " 0x" + Integer.toHexString(uartData[1]));
            return false;
        }
        return true;
    }


    /**
     * Reads an 8 bit value over I2C
     * @param reg the register to read from.
     * @return
     */
    private synchronized byte read8(reg_t reg) {
        readLen(reg, uartByteBuf);
        return uartByteBuf[0];
    }

    /**
     * Reads the specified number of bytes over I2C
     *
     * @param reg the address to read from
     * @param buffer to store the read data into
     * @return true on success
     */
    private boolean readLen(reg_t reg, byte[] buffer) {
        return readLen(reg.getVal(), buffer);
    }

    private boolean uartRead(byte[] buffer, int offset, int length)
    {
        int lenRead = 0;
        int timeout = 50;
        for(;;)
        {
            int ret = uart.read(buffer, offset+lenRead, length - lenRead);
            if (ret < 0) return false;
            if (ret == 0)
            {
                Delay.msDelay(1);
                if (timeout-- <= 0) return false;
            }
            lenRead += ret;
            if (lenRead >= length) return true;
        }
    }
    
    /**
     * Reads the specified number of bytes over I2C
     *
     * @param reg the address to read from
     * @param buffer the size of the data to read
     * @return true on success
     */
    private synchronized boolean readLen(int reg, byte[] buffer) {
        if (buffer == null || buffer.length < 1) {
            debug("Invalid length received.");
            return false;
        }
        byte[] uartData = uartIOBuf;
        for(;;)
        {
            while (uart.read(uartData, 0, uartData.length) > 0)
                Delay.msDelay(1);
            uartData[0] = (byte)0xAA;
            uartData[1] = (byte)1;
            uartData[2] = (byte)reg;
            uartData[3] = (byte)buffer.length;
            uart.write(uartData, 0, 4);
            if (!uartRead(uartData, 0, 2)) return false;
            debug("got response 0x" + Integer.toHexString(uartData[0]));
            debug("got response 0x" + Integer.toHexString(uartData[1]));
            if (uartData[0] == (byte)0xBB)
            {
                if (uartData[1] != buffer.length)
                    System.out.println("Data error length does not match wanted " + buffer.length + " got " + (int)uartData[1]);
                uartRead(buffer, 0, buffer.length);
            }
            else
            {
                //System.out.println("ret " + Integer.toHexString(uartData[0]) + " code " + Integer.toHexString(uartData[1]));
                if (uartData[0] == (byte) 0xee && uartData[1] == (byte)7)
                {
                    //System.out.println("Retry");
                    continue;
                }
            }
            break;
        }
        /*
        try {
            getData(reg, buffer, buffer.length);
        }
        catch (I2CException e)
        {
            System.out.println("Got exception");
            return false;
        }
        */
        debug("Reading data from 0x" + Integer.toHexString(reg));
        for(int i = 0; i < buffer.length; i++) {
            debug("  Read[" + i + "]: 0x" + Integer.toHexString(buffer[i]));
        }

        return true;
    }


    private void debug(String val) {
        if (DEBUG) {
            System.out.println(val);
        }
    }
}
