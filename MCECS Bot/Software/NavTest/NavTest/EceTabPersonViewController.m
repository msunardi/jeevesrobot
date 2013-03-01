//
//  EceTabPersonViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/17/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "EceTabPersonViewController.h"

@interface EceTabPersonViewController () {
    
    }

@end

@implementation EceTabPersonViewController

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
    @try {
        colours = [[NSArray alloc] initWithObjects:
                   [UIColor redColor],
                   [UIColor blueColor],
                   [UIColor greenColor],
                   [UIColor magentaColor],
                   [UIColor yellowColor],
                   [UIColor whiteColor],
                   [UIColor grayColor],
                   [UIColor lightGrayColor],
                   [UIColor purpleColor],
                   [UIColor orangeColor],
                   nil];        
        
        self.title = @"DTGridView";
        // Just another way to do the above ...
        self.gridView = [[DTGridView alloc]initWithFrame:self.view.bounds];
        self.gridView.delegate = self;
        self.gridView.dataSource = self;
        self.gridView.bounces = YES;        
         
    }
    @catch (NSException *exception) {
        NSLog(@"Exception: %@",exception);
    }
    @finally {
        return;
    }
    
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


    
#pragma mark -
#pragma mark DTGridViewDataSource methods

- (NSInteger)numberOfRowsInGridView:(DTGridView *)gridView {
	return 25;
}
- (NSInteger)numberOfColumnsInGridView:(DTGridView *)gridView forRowWithIndex:(NSInteger)index {
	return 25;
}

- (CGFloat)gridView:(DTGridView *)gridView heightForRow:(NSInteger)rowIndex {
	return 100.0;
}
- (CGFloat)gridView:(DTGridView *)gridView widthForCellAtRow:(NSInteger)rowIndex column:(NSInteger)columnIndex {
	return 100.0;
}

- (DTGridViewCell *)gridView:(DTGridView *)gv viewForRow:(NSInteger)rowIndex column:(NSInteger)columnIndex {
	
    @try {
    
	DTGridViewCell *cell = [gv dequeueReusableCellWithIdentifier:@"cell"];
    
	if (!cell) {
		cell = [[DTGridViewCell alloc] initWithReuseIdentifier:@"cell"];
	}
	
	cell.backgroundColor = [colours objectAtIndex:(random() % 10)];
	
	return cell;
    }
    @catch (NSException *exception) {
        NSLog(@"gridView:viewForRow:column: error: %@",exception);
    }

}

#pragma mark -
#pragma mark DTGridViewDelegate methods

- (void)gridView:(DTGridView *)gv selectionMadeAtRow:(NSInteger)rowIndex column:(NSInteger)columnIndex {
	NSLog(@"%@:%@ %@", self, NSStringFromSelector(_cmd), [gv cellForRow:rowIndex column:columnIndex]);
	
}

- (void)gridView:(DTGridView *)gridView scrolledToEdge:(DTGridViewEdge)edge {
}


- (void)viewDidUnload {
    [self setGridView:nil];
    [super viewDidUnload];
}
@end
