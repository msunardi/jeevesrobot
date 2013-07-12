package marion;
import java.io.*;
import java.util.*;



public class Marion {

	private Map<String, String> pos;
	private Map<String, ProfList> mapping;
	private Map<String, Integer> locations;
	private Map<String, String[]> grammar;
	
	public Marion() {
		Scanner scan = null;
		String str;
		pos = new HashMap<String, String>();
		grammar = new TreeMap<String, String[]>();
		//initialize parts of speech mappings (go is a verb, bike is a noun, etc.)
		try {
			scan = new Scanner(new File("/home/mcecsbot/workbenches/Marion/src/pos.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		while(scan.hasNext()) {	
			str = scan.next();
			pos.put(str, scan.nextLine().trim());
		}
		scan.close();
		
		//SQL integration failed 
		
		mapping = new TreeMap<String, ProfList>();
		try {
			scan = new Scanner(new File("/home/mcecsbot/workbenches/Marion/src/Professors.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		String[] strA;
		while(scan.hasNextLine()) {
			str = scan.nextLine();
			strA = str.split(",");
			ProfList prof = new ProfList(strA);
			for (String s: strA) {
				mapping.put(s, prof);
			}
		}
		scan.close();
		
		
		//locations 
		
		locations = new TreeMap<String, Integer>();
		try {
			scan = new Scanner(new File("/home/mcecsbot/workbenches/Marion/src/LID.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		while(scan.hasNextLine()) {
			str = scan.nextLine();
			strA = str.split(",");
			locations.put(strA[0], Integer.parseInt(strA[1]));
		}
		scan.close();
		
		//scan in grammar rules (a sentence is a nounphrase followed by a verbphrase, etc.)
		try {
			scan = new Scanner(new File("/home/mcecsbot/workbenches/Marion/src/grammar.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		while(scan.hasNext()) {
			str = scan.next();
			grammar.put(str, scan.nextLine().trim().split("\\|"));
		}
		scan.close();
	}
	
	//creates a list of SUnits (syntactical units)
	//words are single nodes, phrases are trees with words as leaf nodes
	public String comprehend(String sentence) {
		Scanner scan = new Scanner(sentence);
		List<SUnit> list = new LinkedList<SUnit>();
		String name;
		String response = "comprehend()";
		while(scan.hasNext()) {
			name = scan.next();
			System.out.println(name);
			SUnit neo = new Word(name, pos.get(name)); //creates word objects
			if(neo.type==null) {
				wordError(name); //when present with a word that it does not know, displays info about the unknown word (not an exception)
				return "I don't get it";
			}
			list.add(neo);
		}
		scan.close();
		
		
		boolean status = match(list, 0, 0);
		if(status&&list.get(0).type.equals("verbphrase")) { //implicitly makes a command
			list.add(0, new Word("Marion", "marion"));
			status = match(list, 0, 0);
		}
		if(status){
			if (list.size() > 1) {
				response = understand((Phrase) list.get(0));//if the program understands the sentence
			} else {
				String fs;
				fs = String.format("What about %s?", list.get(0).toString());
				System.out.println(fs);
				return fs;
			}
		} else {
			grammarError(); //if it doesn't
			return "grammar error.";
		}
		return response;
	}

	private String understand(Phrase phrase) { //very minimal will need to be vastly expanded 
											//for this to actually understand meaning rather than just structure
		String response = "understand()";
		System.out.println("You sentence is a "+ phrase.type+".");
		if(phrase.type.equals("sentence")) {
			System.out.println("This is an indicative statement. I'm going to ignore this.");
			response = "This is an indicative statement. I'm going to ignore this.";
		}else if(phrase.type.equals("command")) {
			System.out.println("This is a command.");
				Word verb = findVerb(phrase);
				System.out.println(perform(verb, phrase));
				response = perform(verb, phrase);
		}else {
			System.out.println("This is a question.");
			if(questionType(phrase).equals("Where")) {
				find(phrase);
				response = "some question";
			}
		}
		return response;
	}

	private void find(SUnit phrase) {
		while(mapping.get(phrase.toString()) == null) {
			phrase = ((Phrase)phrase).links.get(1);
		}
		
	}

	private String perform(Word verb, SUnit phrase) {
		if (verb.name.equals("go")) {
			while (!phrase.type.equals("nounphrase") || locations.get(phrase.toString()) == null) {
				phrase = ((Phrase)phrase).links.get(1);
			}
			return("I am on my way to Location " + locations.get(phrase.toString()));
		}
		return "I do not know how to do that.";
	}

	private Word findVerb(SUnit current) {
		current = ((Phrase)current).links.get(1);
		while(current instanceof Phrase) {
			current = ((Phrase)current).links.get(0);
		}
		return (Word) current;
	}

	private String questionType(SUnit current) {
		while(current instanceof Phrase) {
			current = ((Phrase)current).links.get(0);
		}
		return((Word)current).name;
	}

	private void wordError(String name) { //word not in lexicon
		System.out.println("I'm sorry, I don't understand what "+name+" means.");
	}

	private void grammarError() {//the program cannot create a sentence out of the given words
		System.out.println("I'm sorry, I understand the words, but not the sequence that you used them in.");
		
	}

	private boolean match(List<SUnit> list, int start, int status) { //the brains of the program
		System.out.println("match()-ing ...");
		if(list.size() == 1) {//case when entire sentence is reduced to one phrase
			return true;
		}
		//out of bounds checks
		if(start <0) {
			return false;
		}
		if (start >= list.size()) {
			return false;
		}
		for(String s: grammar.keySet()) {//cycle through every phrase
			for(String rules: grammar.get(s)) {//cycle through every rule for a phrase
				String[] targets = rules.split(" "); //convert rules into a usable form
				/*for (int l = 0; l < targets.length; l++) {
					System.out.println(targets[l]);
				}*/
				if(targets.length > list.size()-start) {//check if sentence is longer than rule
					continue; //if so, go to next rule, or next phrase if at last rule for a phrase
				}
				int i = start; //where in the list we are looking
				SUnit current;
				boolean hack=false;
				for(String type: targets) {//go though the rule, see if the next elements in the list match the rule
					current = list.get(i);
					System.out.println(String.format("%s: %s", type, current.toString()));
					if(!current.type.equals(type)) {
						hack=true;
						break;//if not a match, go to next rule or phrase
					}
					i++;
				}
				if (hack) {
					continue;
				}
				//a match was found!
				SUnit test = new Phrase(s); //create a phrase 
				for (int j = start; j < i; j++) {
					((Phrase) test).add(list.remove(start)); //remove words/phrases from the list and put them in the phrase
				}
				list.add(start, test); //add the phrase to the place where we started at
				boolean result;
				if (status==0) {
					result = match(list, start, 0);
				}else if(status==1) {
					result = match(list, start-1, -1);
				}else if(start!=0){//backtracking and not at beginning
					result = match(list, start-1, -1);
				} else{//backtracking and at beginning
					result = match(list, start, 0);
				}
				if (result) {//got a match
					return true;
				}else {
					undo(list, start);
				}
			}
		}
		//no match found
		if(status == -1) {
			return match(list, start-1, -2);
		}else if(status == -2) {//backtracking, no change therefore no change previously either
			return false;
		}else{
			return match(list, start+1, 1);
		}
	}
		

	private void undo(List<SUnit> list, int start) {
		Phrase current = (Phrase) list.remove(start);
		for(SUnit link: current.links) {
			list.add(start, link);
			start++;
		}
	}
}