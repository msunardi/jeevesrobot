#from django.test import TestCase
from django.test import LiveServerTestCase
from selenium import webdriver

class NavigationTest(LiveServerTestCase):
    def setUp(self):
        self.browser = webdriver.Firefox()
        self.browser.implicitly_wait(3)

    def tearDown(self):
        self.browser.quit()

    def test_can_load_page(self):
        self.browser.get(self.live_server_url + '/navigation')
        body = self.browser.find_element_by_tag_name('body')
        self.assertIn('Navigation', body.text)
        self.fail('Finish this test')
# Create your tests here.
