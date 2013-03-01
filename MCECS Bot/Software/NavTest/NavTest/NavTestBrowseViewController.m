//
//  NavTestBrowseViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/12/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "NavTestBrowseViewController.h"

@interface NavTestBrowseViewController ()
{
    NSMutableArray *personArray;
    sqlite3 *personDB;
    NSString *dbPathString;
}

@end

@implementation NavTestBrowseViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    personArray = [[NSMutableArray alloc]init];
    [[self myTableView]setDelegate:self];
    [[self myTableView]setDataSource:self];
    [self createOrOpenDB];
}

- (void)createOrOpenDB
{
    NSLog(@"Create or Open DB...");
    NSArray *path = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *docPath = [path objectAtIndex:0];
    
    dbPathString = [docPath stringByAppendingPathComponent:@"persons.db"];
    
    char *error;
    
    NSFileManager *fileManager = [NSFileManager defaultManager];
    
    if (![fileManager fileExistsAtPath:dbPathString]) { // If file doesn't exist ...
        const char *dbPath = [dbPathString UTF8String];
        
        //Create database
        if (sqlite3_open(dbPath, &(personDB)) == SQLITE_OK) {
            NSLog(@"Creating database at %@", dbPathString);
            const char *sql_statement = "CREATE TABLE IF NOT EXISTS PERSONS (ID INTEGER PRIMARY KEY AUTOINCREMENT, NAME TEXT, AGE INTEGER)";
            if (sqlite3_exec(personDB, sql_statement, NULL, NULL, &(error)) == SQLITE_OK) {
                NSLog(@"Success!");
            } else {
                NSLog(@"Failed to build table. Table may already exists.");
            }
        }
        
        sqlite3_close(personDB);
    } // Else, don't worry about it - the file exists
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setNameField:nil];
    [self setAgeField:nil];
    [self setMyTableView:nil];
    [super viewDidUnload];
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
    return [personArray count];
}

- (IBAction)addButton:(id)sender {
    NSLog(@"addPersonButton clicked.");
    
    char *error;
    
    if (sqlite3_open([dbPathString UTF8String], &(personDB)) == SQLITE_OK) {
        NSString *insertStatement = [NSString stringWithFormat:@"INSERT INTO PERSONS(NAME,AGE) VALUES ('%s', '%d')", [self.nameField.text UTF8String], [self.ageField.text intValue]];
        const char *insert_statement = [insertStatement UTF8String];
        
        if (sqlite3_exec(personDB, insert_statement, NULL, NULL, &(error)) == SQLITE_OK) { // If inserting to database is successful ...
            // Add to personsArray
            Person *person = [[Person alloc]init];
            [person setName:self.nameField.text];
            [person setAge:[self.ageField.text intValue]];
            
            [personArray addObject:person];
            
            NSLog(@"Person added");
        } else {
            NSLog(@"Failed to add person.");
        }
        sqlite3_close(personDB);
    }
}

- (IBAction)showAllButton:(id)sender {
    sqlite3_stmt *statement;
    
    if (sqlite3_open([dbPathString UTF8String], &(personDB)) ==  SQLITE_OK) {
        
        [personArray removeAllObjects];
        
        NSString *query = [NSString stringWithFormat:@"SELECT * FROM PERSONS"];
        const char *querystring = [query UTF8String];
        
        if (sqlite3_prepare(personDB, querystring, -1, &(statement), NULL) == SQLITE_OK) {
            while (sqlite3_step(statement) == SQLITE_ROW) {
                NSString *name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 1)];
                NSString *ageString = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 2)];
                
                Person *person = [[Person alloc]init];
                
                [person setName:name];
                [person setAge:[ageString intValue]];
                
                [personArray addObject:person];
            }
        }
    }
    sqlite3_close(personDB);
    [[self myTableView] reloadData];

}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    
    static NSString *CellIdentifier = @"Cell";
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifier];
    
    if (!cell) {
        cell = [[UITableViewCell alloc]initWithStyle:UITableViewCellStyleValue1 reuseIdentifier:CellIdentifier];
    }
    
    Person *aPerson = [personArray objectAtIndex:indexPath.row];
    
    cell.textLabel.text = aPerson.name;
    cell.detailTextLabel.text = [NSString stringWithFormat:@"%d", aPerson.age];
    
    return cell;
}

- (IBAction)deleteButton:(id)sender {
    [[self myTableView]setEditing:!self.myTableView.editing animated:YES];
}

- (IBAction)nextButton:(id)sender {
}

- (void)tableView:(UITableView *)tableView commitEditingStyle:(UITableViewCellEditingStyle)editingStyle forRowAtIndexPath:(NSIndexPath *)indexPath {
    if (editingStyle == UITableViewCellEditingStyleDelete) {
        Person *p = [personArray objectAtIndex:indexPath.row];
        
        [self deleteData:[NSString stringWithFormat:@"DELETE FROM PERSONS WHERE NAME IS '%s'", [p.name UTF8String]]];
        [personArray removeObjectAtIndex:indexPath.row];
        [tableView deleteRowsAtIndexPaths:[NSArray arrayWithObject:indexPath] withRowAnimation:UITableViewRowAnimationFade];
    }
}

- (void)deleteData:(NSString *)deleteQuery {
    char *error;
    
    if (sqlite3_exec(personDB, [deleteQuery UTF8String], NULL, NULL, &(error)) == SQLITE_OK) {
        NSLog(@"Person deleted");
    } else {
        NSLog(@"Failed to delete: %s", error);
    }
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    if ([segue.identifier isEqualToString:@"showDetailSegue"]) {
        NSLog(@"Segue identifier is %@",segue.identifier);
        //UINavigationController *navController = (UINavigationController *) segue.destinationViewController;
        DetailViewController *controller = (DetailViewController *)segue.destinationViewController;
        controller.somePerson = [[Person alloc]init];
        controller.somePerson.name = [[personArray objectAtIndex:1]name];
        controller.somePerson.age = [[personArray objectAtIndex:1]age];
        
    }
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    [super touchesBegan:touches withEvent:event];
    [[self ageField]resignFirstResponder];
    [[self nameField]resignFirstResponder];
}

@end
