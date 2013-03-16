#include <stdio.h>
#include <string.h>
#include <regex.h> 
#include <stdlib.h>
int match(char [], char []);
 
int main() {
  char a[100], b[100];
  int position;
  regex_t regex;
  int reti;
  char msgbuf[100];
 
  printf("Enter some text\n");
  gets(a);
 
  printf("Enter a string to find\n");
  gets(b);
 
  position = match(a, b);
 
  if(position != -1) {
    printf("Found at location %d\n", position + 1);
  }
  else {
    printf("Not found.\n");
  }

/* Compile regular expression */
        reti = regcomp(&regex, "^a[[:alnum:]]", 0);
        if( reti ){ fprintf(stderr, "Could not compile regex\n"); exit(1); }

/* Execute regular expression */
        reti = regexec(&regex, "bc", 0, NULL, 0);
        if( !reti ){
                puts("Match");
        }
        else if( reti == REG_NOMATCH ){
                puts("No match");
        }
        else{
                regerror(reti, &regex, msgbuf, sizeof(msgbuf));
                fprintf(stderr, "Regex match failed: %s\n", msgbuf);
                exit(1);
        }

/* Free compiled regular expression if you want to use the regex_t again */
    regfree(&regex); 
  return 0;
}
 
int match(char text[], char pattern[]) {
  int c, d, e, text_length, pattern_length, position = -1;
 
  text_length    = strlen(text);
  pattern_length = strlen(pattern);
 
  if (pattern_length > text_length) {
    return -1;
  }
 
  for (c = 0; c <= text_length - pattern_length; c++) {
    position = e = c;
 
    for (d = 0; d < pattern_length; d++) {
      if (pattern[d] == text[e]) {
        e++;
      }
      else {
        break;
      }
    }
    if (d == pattern_length) {
      return position;
    }
  }
 
  return -1;
}
