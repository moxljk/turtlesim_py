import os
__here = __file__
IMAGES_DIR = os.path.join(os.path.dirname(os.path.dirname(__here)), "images/")

if __name__ == "__main__":
    print(IMAGES_DIR)