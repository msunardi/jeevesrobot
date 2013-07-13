package speechrecognition;

import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.recognizer.Recognizer;
//import edu.cmu.sphinx.result.ConfidenceResult;
//import edu.cmu.sphinx.result.ConfidenceScorer;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import java.util.Scanner;
import com.sun.speech.freetts.Voice;
import com.sun.speech.freetts.VoiceManager;
import java.io.*;
import java.net.*;
import java.util.*;
import org.apache.commons.lang3.*;
import marion.Marion;


public class SpeechRecognizer {
	public static void main(String[] args) {

        Voice voice;
        VoiceManager voiceManager = VoiceManager.getInstance();
             
        voice = voiceManager.getVoice("kevin16");
        voice.allocate();
	
	
		//Set up microphone and speech recognizer
        ConfigurationManager cm;
        if (args.length > 0) {
            cm = new ConfigurationManager(args[0]);
        } else {
            cm = new ConfigurationManager(SpeechRecognizer.class.getResource("helloworld.config.xml"));
        }
        System.out.println("Startup...");
        Recognizer recognizer = (Recognizer) cm.lookup("recognizer");
        recognizer.allocate();
        // start the microphone or exit if the program if this is not possible
        Microphone microphone = (Microphone) cm.lookup("microphone");
        if (!microphone.startRecording()) {
            System.out.println("Cannot start microphone.");
            recognizer.deallocate();
            System.exit(1);
        }
	
        //ConfidenceScorer scorer = (ConfidenceScorer) cm.lookup("confidencescorer");
        
        //System is running
        System.out.println("What would you like to know?");
        
        //Socket communication
        Socket kkSocket = null;
        PrintWriter out = null;
        BufferedReader in = null;
        try {
            //kkSocket = new Socket("131.252.166.173", 80);
        	kkSocket = new Socket("127.0.0.1", 8008);
            out = new PrintWriter(kkSocket.getOutputStream(), true);
            in = new BufferedReader(new InputStreamReader(kkSocket.getInputStream()));
            out.println("iam:speech\n");
            String servermsg = in.readLine();
            System.out.println("blah:"+servermsg);            
                        
        } catch (UnknownHostException e) {
            System.err.println("Don't know about host");
            //System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to host");
            System.exit(1);
        }
        
        
        //String fileName = "resources/responses.csv";
        //File file = new File(fileName);
        File flea = new File("resources/responses.csv");
        Scanner flearead;
        Map <String, String> response_map = new HashMap<String, String>();
		try {
			flearead = new Scanner(flea);
			
			while (flearead.hasNext()) {
	        	String data = flearead.nextLine();
	        	String[] values = data.split(";");
	        	String tempval;
	        	if (values.length > 2) {
	        		tempval = StringUtils.join(new String[] {values[1],values[2]}, ";");   		
	        	} else {
	        		tempval = values[1]; 
	        	}
	        	response_map.put(values[0], tempval);
	        	
	        }
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
        
        
        
        Boolean blah;
        Marion marion = new Marion();
        // loop the recognition until the programm exits.
		while (true) {
			blah = false;
			System.out.print(" ... ");
			/*try {
	        	out.println("iam:speech");
	        	System.out.println(in.readLine());
	        } catch (Exception e) {
	        	System.out.println(e.toString());
	        }*/
			//BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in));
			//Get result input from speech
			
			Scanner scan = new Scanner(System.in);
			Result result = null;
			String resultText = null;
			int choice;
			do {
				System.out.println("1. Enter a sentence\t\t0. Quit");
				choice = Integer.parseInt(scan.nextLine());
				if(choice == 1) {
					System.out.println("Type in the sentence");
					String marionresponse = marion.comprehend(scan.nextLine().toLowerCase());
					System.out.println(marionresponse);
				} else if (choice == 2) {
					result = recognizer.recognize();
					if (result != null) {
						resultText = result.getBestFinalResultNoFiller();
						break;
					}
					//else continue;
					/*int min = 0, max = 4;
					Random rand = new Random();
					int randomnum = rand.nextInt(max - min + 1) + min;
					System.out.println(randomnum);
					String [] commands = {"lean left", "can you wave", "stand up straight", "hi robot", "see you later"};
					resultText = commands[randomnum];
					break;
					*/
				}
			} while(choice!=0);
			
            //Result result = recognizer.recognize();
            
            
			//If there is a result
            //if (result != null) {
			if (resultText != null) {
				//Get String of result
                //String resultText = result.getBestFinalResultNoFiller();
            	//String resultText = result.getBestResultNoFiller();
                //java.lang.String toString = (resultText);
                //System.out.print(String.format("Recognized speech: %s\n", toString));
				System.out.print(String.format("Recognized speech: %s\n", resultText));
                
                //////////////// Using hashmap response_map //////////////////////
                try {
                	// Trying to find the map element with key=resultText
                	String response = response_map.get(resultText).toString();
                	
                	String [] response_list = response.split(";");
                	System.out.println(response_list[0]);
                	if (response_list.length > 1) {
                		try {
                			out.println(response_list[1]);
                			System.out.println(in.readLine());
                		} catch (Exception e) {
                			System.out.println("Failed sending to socket");
                		}
                	}
                	blah=true;
                } catch (Exception e) {
                	System.out.println("Sorry, I didn't quite get that.");
                }
                //////////////////////////////////////////////////////////////////
              
                
				//Search CSV for response and separate the values
				//value[0] == the speech your looking for
				//value[1] == the response
				//value[2] == the command to the main thread
	            //String fileName = "resources/responses.csv";
	            //File file = new File(fileName);
	            //int connect = 1;
	            //try{
	    	    //    Scanner inputStream = new Scanner(file);
	    	        
	    	        //Socket communication is commented out for testing
	    	        /*
    	        		Socket kkSocket = null;
    	                PrintWriter out = null;
    	                BufferedReader in = null;
    	                try {
    	                    //kkSocket = new Socket("131.252.166.173", 80);
    	                	kkSocket = new Socket("localhost", 8008);
    	                    out = new PrintWriter(kkSocket.getOutputStream(), true);
    	                    in = new BufferedReader(new InputStreamReader(kkSocket.getInputStream()));
    	                    System.out.println(in.readLine());
    	                } catch (UnknownHostException e) {
    	                    System.err.println("Don't know about host");
    	                    //System.exit(1);
    	                } catch (IOException e) {
    	                    System.err.println("Couldn't get I/O for the connection to host");
    	                    //System.exit(1);
    	                }
    	                out.println("iam:speech");
    	             */   
    	                
	    	        
	    	        
	    	        /*while (inputStream.hasNext()){
	    	        	blah = true;
	    	        	//System.out.println("inputstream.hasnext");
	    	        	String data = inputStream.nextLine();
	    	        	System.out.println(data);
	    	        	String[] values = data.split(";");
	    	        	
	    	        	System.out.print(String.format("Key: %s\n", values[0]));
	    	        	if(resultText.equals(values[0])){
							//Print out Response
	    	        		System.out.print("Response: ");
	    	        		if ((values[1] == "") || (values[1] == null)) {
	    	        			System.out.println("empty or null");
	    	        		} else {
	    	        			System.out.println(values[1]);
	    	        		}
	    	        		*/


							//Send command to socket
							//out.println(values[2]);
							
	    	        		//Speak the response
	    	        		//voice.speak(values[1]);
	    	        		//voice.speak("Foo bar!".toString());
	    	        		
							//Close the Socket
							/*
							out.close();
							in.close();
							stdIn.close();
							kkSocket.close();*/
							
	    	        	/*}
	    	  	
	    	        	
	    	        }
	    	        inputStream.close();
	            } catch(FileNotFoundException e) {
	            	e.printStackTrace();
	            }*//*catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}*/
            }
            else {
                System.out.println("I can't hear what you said.\n");
            }
            if (blah) {
            	System.out.println("Yes it got something");
            } else System.out.println("No it didn't catch anything");
        } 
		 
    }
}
