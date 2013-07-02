/*
 * Main Speech program for CECS Bot Winter 2013
 * Written by Richard Bernard
 * 
 * Takes in speech from user, if it is a matched phrase it will respond back with a pre-dertermined response
 * Also sends a pre-determined command code to main Robot which helps integration for such things as navigation
 * by commands etc. 
 *
 * Based on Examples from the Examples given with the Carnegie Mellon University Sphinx Speech Toolkit
 */
 
package edu.cmu.sphinx.demo.helloworld;


import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.result.ConfidenceResult;
import edu.cmu.sphinx.result.ConfidenceScorer;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import java.util.Scanner;
import com.sun.speech.freetts.Voice;
import com.sun.speech.freetts.VoiceManager;
import java.io.*;
import java.net.*;

import com.sun.speech.freetts.Voice;
import com.sun.speech.freetts.VoiceManager;

public class HelloWorld {

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
            cm = new ConfigurationManager(HelloWorld.class.getResource("helloworld.config.xml"));
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
	
        ConfidenceScorer scorer = (ConfidenceScorer) cm.lookup("confidencescorer");
        
        //System is running
        System.out.println("What would you like to know?");

        // loop the recognition until the programm exits.
		while (true) {
			BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in));
			//Get result input from speech
            Result result = recognizer.recognize();
           
            
			//If there is a result
            if (result != null) {
				//Get String of result
                String resultText = result.getBestFinalResultNoFiller();
                java.lang.String toString = (resultText);
				//Search CSV for response and separate the values
				//value[0] == the speech your looking for
				//value[1] == the response
				//value[2] == the command to the main thread
	            String fileName = "responses.csv";
	            File file = new File(fileName);
	            int connect = 1;
	            try{
	    	        Scanner inputStream = new Scanner(file);
	    	        
	    	        //Socket communication is commented out for testing
	    	        /* 
    	        		Socket kkSocket = null;
    	                PrintWriter out = null;
    	                BufferedReader in = null;
    	                try {
    	                    kkSocket = new Socket("131.252.166.173", 80);
    	                    out = new PrintWriter(kkSocket.getOutputStream(), true);
    	                    in = new BufferedReader(new InputStreamReader(kkSocket.getInputStream()));
    	                } catch (UnknownHostException e) {
    	                    System.err.println("Don't know about host");
    	                    //System.exit(1);
    	                } catch (IOException e) {
    	                    System.err.println("Couldn't get I/O for the connection to host");
    	                    //System.exit(1);
    	                }
    	                out.println("iam:speech");
    	                */
    	                
	    	        
	    	        
	    	        while (inputStream.hasNext()){
	    	        	String data = inputStream.nextLine();
	    	        	String[] values = data.split(";");
	    	        	if(resultText.equals(values[0])){
							//Print out Response
	    	        		System.out.println(values[1]);



							//Send command to socket
							//out.println(values[2]);
							
	    	        		//Speak the response
	    	        		voice.speak(values[1]);
	    	        		
							//Close the Socket
							/*
							out.close();
							in.close();
							stdIn.close();
							kkSocket.close();*/
							
	    	        	}
	    	  	
	    	        	
	    	        }
	    	        inputStream.close();
	            } catch(FileNotFoundException e) {
	            	e.printStackTrace();
	            } catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
            }
            else {
                System.out.println("I can't hear what you said.\n");
            }
        } 
        
        
   
        	
       
		 
    }
}

