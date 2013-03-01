//
//  EceTabPersonGridViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/22/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "EceTabPersonGridViewController.h"

@interface EceTabPersonGridViewController () {
    NSMutableArray *facultyArray;
    sqlite3 *facultyDB;
    NSString *dbPathString;

}

@end

@implementation EceTabPersonGridViewController
@synthesize gridView;

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
    facultyArray = [[NSMutableArray alloc]init];
    
    self.gridView = [[AQGridView alloc] initWithFrame:CGRectMake(0, 0, 768, 1024)];
    self.gridView.autoresizingMask = UIViewAutoresizingFlexibleWidth|UIViewAutoresizingFlexibleHeight;
    self.gridView.autoresizesSubviews = YES;
    self.gridView.delegate = self;
    self.gridView.dataSource = self;
    
    //self.gridView.leftContentInset = 2.0;
    //self.gridView.rightContentInset = 2.0;
    //[self.gridView setResizesCellWidthToFit:YES];
    //[self.gridView setSeparatorStyle:AQGridViewCellSelectionStyleGreen];
    
    UIImage *backgroundPattern = [UIImage imageNamed:@"fibonacci.png"];
    [self.view setBackgroundColor:[UIColor colorWithPatternImage:backgroundPattern]];
    
    // Load database
    [self createEditableCopyOfDatabaseIfNeeded];
    [self.view addSubview:gridView];
    [self.gridView reloadData];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark database methods ...
// Create or load database
- (void)createEditableCopyOfDatabaseIfNeeded {
    NSLog(@"HEY HEY!");
    BOOL success;
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSError *error;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    dbPathString = [documentsDirectory stringByAppendingPathComponent:@"ece.db"];
    success = [fileManager fileExistsAtPath:dbPathString];
    if (success) {
        NSLog(@"Faculty database loaded!");
        [self populateTable];
        return;
    }
    // The writable database does not exist, so copy the default to the appropriate location.
    NSString *defaultDBPath = [[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"ece.db"];
    success = [fileManager copyItemAtPath:defaultDBPath toPath:dbPathString error:&error];
    if (!success) {
        NSAssert1(0, @"Failed to create writable database file with message '%@'.", [error localizedDescription]);
    }
}

// get all data from database
- (void)populateTable {
    sqlite3_stmt *statement;
    
    if (sqlite3_open([dbPathString UTF8String], &(facultyDB)) == SQLITE_OK) {
        NSLog(@"Loading database...");
        [facultyArray removeAllObjects];
        
        NSString *query = [NSString stringWithFormat:@"SELECT id, first_name, middle_name, last_name, image FROM faculty"];
        const char *queryString = [query UTF8String];
        
        if (sqlite3_prepare(facultyDB, queryString, -1, &(statement), NULL) == SQLITE_OK) {
            while (sqlite3_step(statement) == SQLITE_ROW) {
                //int *faculty_id = [(const char *)sqlite3_column_text(statement, 0);
                NSString *first_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 1)];
                NSString *middle_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 2)];
                NSString *last_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 3)];
                NSString *image = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 4)];
                
                Faculty *faculty = [[Faculty alloc]init];
                //NSLog(@"website: %@, %d", website, [website length]);
                /*if (website == @"" || [website length] == 0) {
                    NSLog(@"is empty");
                    website = @"Unavailable";
                }*/
                
                //[faculty setFaculty_id:faculty_id];
                if ([middle_name length] == 0) {
                    [faculty setFull_name:[[NSString alloc] initWithFormat:@"%@ %@", first_name, last_name]];
                } else {
                    [faculty setFull_name:[[NSString alloc] initWithFormat:@"%@ %@ %@", first_name, middle_name, last_name]];
                }

                //[faculty setFull_name:[[NSString alloc] initWithFormat:@"%@ %@", first_name, last_name]];
                [faculty setImage:image];
                
                [facultyArray addObject:faculty];
                NSLog(@"eh!");
            }
        } else {
            NSLog(@"What just happened?");
        }
    } else {
        NSLog(@"Failed to populate!");
    }
}

- (void)getFacultyWithId:(int)facultyId {
    sqlite3_stmt *statement;
    @try {
        if (sqlite3_open([dbPathString UTF8String], &(facultyDB)) == SQLITE_OK) {
            NSLog(@"Loading database...");
            //[facultyArray removeAllObjects];
            
            NSString *query = [NSString stringWithFormat:@"SELECT id, first_name, middle_name, last_name, professor, office, email, website, research_area, image, info, department, other_roles FROM faculty WHERE id=%d", facultyId];
            const char *queryString = [query UTF8String];
            
            if (sqlite3_prepare(facultyDB, queryString, -1, &(statement), NULL) == SQLITE_OK) {
                while (sqlite3_step(statement) == SQLITE_ROW) {
                    //int *faculty_id = [(const char *)sqlite3_column_text(statement, 0);
                    NSString *first_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 1)];
                    NSString *middle_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 2)];
                    NSString *last_name = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 3)];
                    NSString *position = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 4)];
                    NSString *office = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 5)];
                    NSString *email = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 6)];
                    NSString *website = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 7)];
                    NSString *research_area = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 8)];
                    NSString *image = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 9)];
                    NSString *info = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 10)];
                    NSString *department = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 11)];
                    NSString *other_roles = [[NSString alloc]initWithUTF8String:(const char *)sqlite3_column_text(statement, 12)];
                    
                    
                    
                    //Faculty *aFaculty = [[Faculty alloc]init];
                    self.someFaculty = [[Faculty alloc]init];
                    //NSLog(@"website: %@, %d", website, [website length]);
                    /*if (website == @"" || [website length] == 0) {
                     NSLog(@"is empty");
                     website = @"Unavailable";
                     }*/
                    
                    //[faculty setFaculty_id:faculty_id];
                    if ([middle_name length] == 0) {
                        [self.someFaculty setFull_name:[[NSString alloc] initWithFormat:@"%@ %@", first_name, last_name]];
                    } else {
                        [self.someFaculty setFull_name:[[NSString alloc] initWithFormat:@"%@ %@ %@", first_name, middle_name, last_name]];
                    }
                    
                    NSPredicate *predicate;
                    NSString *role = [[NSString alloc]init];
                    predicate = [NSPredicate predicateWithFormat:@"self contains %@", position];
                    
                    if ([predicate evaluateWithObject:@"regular"]) {
                        //[controller.someFaculty setProfessorship:@"Professor"];
                        role = @"Professor";
                    }
                    else if ([predicate evaluateWithObject:@"associate"]) {
                        //[controller.someFaculty setProfessorship:@"Associate Professor"];
                        role = @"Associate Professor";
                    }
                    else if ([predicate evaluateWithObject:@"adjunct"]){
                        //[controller.someFaculty setProfessorship:@"Adjunct Professor"];
                        role = @"Adjunct Professor";
                    }
                    if (([other_roles length] != 0) && ([position length] != 0)) {
                        role = [role stringByAppendingFormat:@", %@", other_roles];
                    } else if (([other_roles length] != 0) && ([position length] == 0)) {
                        role = other_roles;
                    } 
                    NSLog(@"Role: %@",role);
                    [self.someFaculty setProfessorship:[[NSString alloc] initWithFormat:@"%@", role]];
                    
                    [self.someFaculty setOffice:[[NSString alloc] initWithFormat:@"Office: %@",office]];
                    [self.someFaculty setEmail:[[NSString alloc] initWithFormat:@"E-mail: %@",email]];
                    [self.someFaculty setWebsite:[[NSString alloc] initWithFormat:@"Website: %@",website]];
                    [self.someFaculty setResearch_area:[[NSString alloc] initWithFormat:@"Research Area: %@",research_area]];
                    if ([image length] == 0) {
                        image = @"psu_logo.jpg";
                    }
                    [self.someFaculty setImage:image];
                    
                    NSLog(@"Info: %@", info);
                    if ([info length] == 0) {
                        info = @"This person is AWESOME!!! (but seriously, please add his/her info)";
                    }
                    
                    [self.someFaculty setInfo:info];
                    [self.someFaculty setDepartment:[[NSString alloc] initWithFormat:@"Department: %@",department]];
                    
                    //[facultyArray addObject:faculty];
                    NSLog(@"eh! %@, %@", self.someFaculty.full_name, self.someFaculty.image);
                    //return aFaculty;
                }
            } else {
                NSLog(@"What just happened?");
                
            }
        } else {
            NSLog(@"Failed to populate!");
            
        }
        //return nil;
    }
    @catch (NSException *exception) {
        NSLog(@"Failed to get faculty: %@", exception);
    }
    @finally {
        //return nil;
        return;
    }
    

}


// Override to allow orientations other than the default portrait orientation.
- (BOOL) shouldAutorotateToInterfaceOrientation: (UIInterfaceOrientation) interfaceOrientation
{
    return YES;
}

- (void) willRotateToInterfaceOrientation: (UIInterfaceOrientation) toInterfaceOrientation
                                 duration: (NSTimeInterval) duration
{
    if ( UIInterfaceOrientationIsPortrait(toInterfaceOrientation) )
    {
        // width will be 768, which divides by four nicely already
        NSLog( @"Setting left+right content insets to zero" );
        self.gridView.leftContentInset = 0.0;
        self.gridView.rightContentInset = 0.0;
    }
    else
    {
        // width will be 1024, so subtract a little to get a width divisible by five
        NSLog( @"Setting left+right content insets to 2.0" );
        self.gridView.leftContentInset = 2.0;
        self.gridView.rightContentInset = 2.0;
    }
}

#pragma mark AQGridView protocols
- (NSUInteger) numberOfItemsInGridView:(AQGridView *)gridView {
    NSLog(@"%d", [facultyArray count]);
    return [facultyArray count];
}

- (AQGridViewCell *) gridView:(AQGridView *)agridView cellForItemAtIndex:(NSUInteger)index {
    Faculty *aFaculty = [facultyArray objectAtIndex:index];
    static NSString *plainCellIdentifier = @"cell";
    
    GridViewCell *cell = (GridViewCell *)[agridView dequeueReusableCellWithIdentifier:@"cell"];
    if (cell == nil) {
        cell = [[GridViewCell alloc] initWithFrame:CGRectMake(0.0, 0, 240, 240) reuseIdentifier:plainCellIdentifier];
    }
    //NSLog(@"%@", aFaculty.image);
    NSString *image = [[NSString alloc] initWithFormat:@"%@", aFaculty.image];
    if ([image length] == 0) {
        image = @"psu_logo.jpg";
    }
    //[cell.imageView setImage:[UIImage imageNamed:[[NSString alloc] initWithFormat: @"%@", image]]];
    [cell setIcon:[UIImage imageNamed:[[NSString alloc] initWithFormat: @"%@", image]]];
    //[cell.captionLabel setText:@"Sample service"];
    [cell.nameLabel setText:[[NSString alloc] initWithFormat: @"%@",[[facultyArray objectAtIndex:index] full_name]]];
    return cell;
}

- (CGSize) portraitGridCellSizeForGridView:(AQGridView *)gridView {
    return (CGSizeMake(240.0, 240));
}

- (void) gridView:(AQGridView *)aGridView didSelectItemAtIndex:(NSUInteger)index {
    NSLog(@"Selected item at: %u",index);
    self.someFaculty = [[Faculty alloc] init]; // load and prepare the info for the selected faculty (global variable)
    //self.someFaculty = [self getFacultyWithId:index+1]; // <-- stupid indexing difference; database index starts at 1, item/array/datasource b index starts 0
    [self getFacultyWithId:index];
    NSLog(@"Faculty captured: %@", self.someFaculty.full_name);
    [aGridView deselectItemAtIndex:index animated:YES];
    [self performSegueWithIdentifier:@"FacultyDetailView" sender:nil];
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    @try {
        if ([segue.identifier isEqualToString:@"FacultyDetailView"]) {
            NSLog(@"Segue identifier is %@",segue.identifier);
            
            FacultyDetailViewController *controller = (FacultyDetailViewController *)segue.destinationViewController;
            controller.someFaculty = [[Faculty alloc]init];
            NSLog(@"Some Faculty: %@",self.someFaculty.full_name);
            
            [controller.someFaculty setFull_name:self.someFaculty.full_name];
            [controller.someFaculty setDepartment:self.someFaculty.department];
            /*NSPredicate *predicate;
            
            predicate = [NSPredicate predicateWithFormat:@"self contains %@", self.someFaculty.professorship];
            
            if ([predicate evaluateWithObject:@"regular"]) {
                [controller.someFaculty setProfessorship:@"Professor"];
            }
            else if ([predicate evaluateWithObject:@"associate"]) {
                [controller.someFaculty setProfessorship:@"Associate Professor"];
            }
            else if ([predicate evaluateWithObject:@"adjunct"]){
                [controller.someFaculty setProfessorship:@"Adjunct Professor"];
            }*/
            
            [controller.someFaculty setProfessorship:self.someFaculty.professorship];
            [controller.someFaculty setOffice:self.someFaculty.office];
            [controller.someFaculty setEmail:self.someFaculty.email];
            [controller.someFaculty setWebsite:self.someFaculty.website];
            [controller.someFaculty setResearch_area:self.someFaculty.research_area];
            [controller.someFaculty setInfo:self.someFaculty.info];
            [controller.someFaculty setImage:self.someFaculty.image];
            
        }
    }
    @catch (NSException *exception) {
        NSLog(@"prepareForSegue failed: %@", exception);
    }
    @finally {
        return;
    }
    
}

@end
