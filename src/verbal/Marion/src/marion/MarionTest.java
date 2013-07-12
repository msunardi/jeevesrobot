package marion;
import java.util.Scanner;


public class MarionTest {
	public static void main(String[] args) {
		Marion marion = new Marion();
		int choice = -1;
		Scanner scan = new Scanner(System.in);
		do {
			System.out.println("1. Enter a sentence\t\t0. Quit");
			choice = Integer.parseInt(scan.nextLine());
			if(choice == 1) {
				System.out.println("Type in the sentence");
				marion.comprehend(scan.nextLine().toLowerCase());
			}
		} while(choice!=0);
		scan.close();
	}

}
