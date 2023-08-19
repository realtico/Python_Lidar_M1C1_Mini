# Python_Lidar_M1C1_Mini
Graphical interface for a chinese cheap lidar


This program utilizes *pyserial* and *pygame* to provide a graphical view of the lidar output. The lidar used was purchased on AliExpress (https://www.aliexpress.com/snapshot/0.html?spm=a2g0o.9042647.6.2.559e37a1jYP0hF&orderId=8133980903635463&productId=4000251359842) and has an 8m range which is pretty good. 

This is my own development and it was based on a chinese datasheet which I translated to Brazilian Portuguese using Google Translate, since the english version on AliExpress site was incomplete. The code still have a lot of room for improvement. ~~The version here is for windows, but the changes for running on a Raspberry Pi with minimal changes on the  lines in the `if __name__ == '__main__':` block.~~

The latest version is ```Pi_M1C1_Lidar.py``` which is os agnostic and has a better general performance.
