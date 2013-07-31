class SenderThread extends Thread {
  int clientPort = 9100;
  DatagramSocket ds;
  byte[] buffer = new byte[65536];

  boolean running;
  boolean available;
  PImage img;

  SenderThread(int width, int height) {
    img = createImage(width, height, RGB);
    running = false;
    available = true;
  
    try {
      ds = new DatagramSocket(clientPort);
    } catch (SocketException e) {
      e.printStackTrace();
    }
  }

  void setImage(PImage image) {
    img = image;
  }

  void start() {
    running = true;
    super.start();
  }

  void run() {
    while(running) {
      // We need a buffered image to do the JPG encoding
      //img = get(0,0,width,height);
      
      BufferedImage bimg = new BufferedImage( img.width,img.height, BufferedImage.TYPE_INT_RGB );
    
      // Transfer pixels from localFrame to the BufferedImage
      img.loadPixels();
      bimg.setRGB( 0, 0, img.width, img.height, img.pixels, 0, img.width);
    
      // Need these output streams to get image as bytes for UDP communication
      ByteArrayOutputStream baStream  = new ByteArrayOutputStream();
      BufferedOutputStream bos    = new BufferedOutputStream(baStream);
    
      // Turn the BufferedImage into a JPG and put it in the BufferedOutputStream
      // Requires try/catch
      try {
        ImageIO.write(bimg, "jpg", bos);
      } 
      catch (IOException e) {
        e.printStackTrace();
      }
    
      // Get the byte array, which we will send out via UDP!
      byte[] packet = baStream.toByteArray();
    
      // Send JPEG data as a datagram
      println("Sending datagram with " + packet.length + " bytes");
      try {
        print("Trying to send...");
        //ds.send(new DatagramPacket(packet,packet.length, InetAddress.getByName("localhost"),clientPort));
        ds.send(new DatagramPacket(packet,packet.length, InetAddress.getByName("131.252.240.131"),clientPort));
        println("Success");
      } 
      catch (IOException e) {
        e.printStackTrace();
      }
      /*try {
        sleep((long)(10));
      } catch (Exception e) {
        e.printStackTrace();
      }*/
    }
  }
}
