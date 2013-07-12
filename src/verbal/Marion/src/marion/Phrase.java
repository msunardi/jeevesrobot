package marion;
import java.util.*;


public class Phrase extends SUnit {

	public List<SUnit> links;
	
	public Phrase(String s) {
		this.type = s;
		links = new LinkedList<SUnit>();
	}
	
	public void add(SUnit next) {
		links.add(next);
	}
	
	public String toString() {
		String s = "";
		for (SUnit su: links) {
			s+=su.toString();
		}
		return s.trim();
	}

}
