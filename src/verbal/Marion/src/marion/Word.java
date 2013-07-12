package marion;
public class Word extends SUnit{
	public String name;
	
	
	
	public Word(String name, String type) {
		this.name = name;
		this.type = type;
	}
	
	public String toString() {
		return name+" ";
	}
}
