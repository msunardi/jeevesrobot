//
//  EceDatabaseViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "EceDatabaseViewController.h"

@interface EceDatabaseViewController () {
    
    NSMutableArray *labArray;
    sqlite3 *labDB;
    NSString *dbPathString;

}

@end

@implementation EceDatabaseViewController

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
    //self.someLab = [[EceLabs alloc]init];
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    labArray = [[NSMutableArray alloc]init];
    [[self eceTableView]setDelegate:self];
    [[self eceTableView]setDataSource:self];
    //[self loadDB];
    [self createEditableCopyOfDatabaseIfNeeded];
}

- (void)loadDB
{
    NSLog(@"Loading ECE DB ...");
    NSArray *path = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *docPath = [path objectAtIndex:0];
    
    dbPathString = [docPath stringByAppendingPathComponent:@"test.db"];
    
    //char *error;
    
    NSFileManager *fileManager = [NSFileManager defaultManager];
    
    if (![fileManager fileExistsAtPath:dbPathString]) {
        NSLog(@"Cannot find database file!");
    } else {
        NSLog(@"Success.");
        [self populateTable];
    }
}

// Creates a writable copy of the bundled default database in the application Documents directory.
- (void)createEditableCopyOfDatabaseIfNeeded {
    // First, test for existence.
    BOOL success;
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSError *error;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    dbPathString = [documentsDirectory stringByAppendingPathComponent:@"test.db"];
    success = [fileManager fileExistsAtPath:dbPathString];
    if (success) {
        NSLog(@"Success!");
        [self populateTable];
        return;
    }

    // The writable database does not exist, so copy the default to the appropriate location.
    NSString *defaultDBPath = [[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"test.db"];
    success = [fileManager copyItemAtPath:defaultDBPath toPath:dbPathString error:&error];
    if (!success) {
        NSAssert1(0, @"Failed to create writable database file with message '%@'.", [error localizedDescription]);
    }
    
}

- (void)populateTable {
    sqlite3_stmt *statement;
    
    if (sqlite3_open([dbPathString UTF8String], &(labDB)) == SQLITE_OK) {
        
        [labArray removeAllObjects];
        
        NSString *query = [NSString stringWithFormat:@"SELECT * FROM LABS"];
        const char *queryString = [query UTF8String];
        
        if (sqlite3_prepare(labDB, queryString, -1, &(statement), NULL) == SQLITE_OK) {
            while (sqlite3_step(statement) == SQLITE_ROW) {
                NSString *name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 1)];
                NSString *location = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 2)];
                NSString *director = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 3)];
                NSString *website = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 4)];
                NSString *description = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_blob(statement, 5)];
                
                EceLabs *lab = [[EceLabs alloc]init];
                NSLog(@"website: %@, %d", website, [website length]);
                if (website == @"" || [website length] == 0) {
                    NSLog(@"is empty");
                    website = @"Unavailable";
                }
                
                [lab setName:name];
                [lab setRoom:location];
                [lab setDirector:director];
                [lab setWebsite:website];
                [lab setDescription:description];
                
                [labArray addObject:lab];
            }
        }
    }
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section{
    
    return [labArray count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    
    //static NSString *cellIdentifier = @"labCell";
    static NSString *cellIdentifier = @"SampleTableCell";
    SampleTableCell *labCell = [tableView dequeueReusableCellWithIdentifier:cellIdentifier];
    
    if (!labCell) {
        //labCell = [[UITableViewCell alloc]initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:cellIdentifier];
        NSArray *nib = [[NSBundle mainBundle]loadNibNamed:@"SampleTableCell" owner:self options:nil];
        labCell = [nib objectAtIndex:0];
    }
    
    EceLabs *aLab = [labArray objectAtIndex:indexPath.row];
    //labCell.textLabel.text = aLab.name;
    //labCell.detailTextLabel.text = aLab.description;
    labCell.labCellName.text = aLab.name;
    labCell.labCellRoom.text = aLab.room;
    labCell.labCellDescription.text = aLab.description;
    labCell.labCellImage.image = [UIImage imageNamed:@"psu_logo.jpg"];

    return labCell;

}

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath {
    NSLog(@"Did select row at index: %d",indexPath.row);
    /*EceDetailViewController *edView = [self.storyBoard instantiateViewControllerWithIdentifier:@"EceDetailViewController"];
    [self.navController pushViewController:(UIViewController *)edView animated:YES];*/
    
    NSLog(@"somelab: %@, %d",[[labArray objectAtIndex:(int)indexPath.row]name], indexPath.row);
    
    self.someLab = [[EceLabs alloc]init];
    
    [self.someLab setName:[[labArray objectAtIndex:(int)indexPath.row]name]];
    [self.someLab setRoom:[[labArray objectAtIndex:(int)indexPath.row]room]];
    [self.someLab setWebsite:[[labArray objectAtIndex:(int)indexPath.row]website]];
    [self.someLab setDirector:[[labArray objectAtIndex:(int)indexPath.row]director]];
    [self.someLab setDescription:[[labArray objectAtIndex:(int)indexPath.row]description]];
    NSLog(@"someLab name: %@",self.someLab.name);
    [self performSegueWithIdentifier:@"EceDetailSegue" sender:nil];
}

- (CGFloat)tableView:(UITableView *)tableView heightForRowAtIndexPath:(NSIndexPath *)indexPath {
    return 135;
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    if ([segue.identifier isEqualToString:@"EceDetailSegue"]) {
        NSLog(@"Segue identifier is %@",segue.identifier);
        //UINavigationController *navController = (UINavigationController *) segue.destinationViewController;
        EceDetailViewController *controller = (EceDetailViewController *)segue.destinationViewController;
        controller.eceLab = [[EceLabs alloc]init];
        NSLog(@"%@",self.someLab.name);
        
        controller.eceLab = [[EceLabs alloc]init];
        //controller.eceLab.name = self.someLab.name;
        [controller.eceLab setName:self.someLab.name];
        [controller.eceLab setRoom:[NSString stringWithFormat:@"Room: %@",self.someLab.room]];
        [controller.eceLab setWebsite:[NSString stringWithFormat:@"URL: %@",self.someLab.website]];
        [controller.eceLab setDirector:[NSString stringWithFormat:@"Director: %@",self.someLab.director]];
        [controller.eceLab setDescription:self.someLab.description];
        
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setEceTableView:nil];
    [super viewDidUnload];
}
@end
