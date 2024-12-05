from dom import DOM
import numpy as np

# initialize
iqa = DOM()

# using image path
score = iqa.get_sharpness('/home/mark/Downloads/GussianBlur13_Test_Image.jpg')
print("Sharpness:", score)

score = iqa.get_sharpness('/home/mark/Downloads/45degreemotionblur_Test_Image.jpg')
print("Sharpness:", score)

score = iqa.get_sharpness('/home/mark/Downloads/Test_Image.jpg')
print("Sharpness:", score)

# using numpy array
img = np.random.randint(50, size=(10,10,3), dtype=np.uint8)
score = iqa.get_sharpness(img)
print("Sharpness:", score)