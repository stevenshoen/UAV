#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  6 10:05:48 2020

@author: pi
"""

import boto3

"""
Access Key ID:
AKIAJX2Q3ASHIJY6FSTQ
Secret Access Key:
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
"""

#s3 = boto3.client('s3')
s3 = boto3.resource('s3')
obj = s3.Bucket(name='landsat-pds')

for bk in obj.objects.all():
    print(type(bk))
    break
"""
first_bucket = s3_resource.Bucket(name=first_bucket_name)
first_object = s3_resource.Object(
    bucket_name=first_bucket_name, key=first_file_name)

s3_resource.Object(first_bucket_name, first_file_name).download_file(
    f'/tmp/{first_file_name}')

"""
fn = 'L8/001/053/LC80010532013269LGN00/LC80010532013269LGN00_B5.TIF'


with open('bot.bt', '+wb') as f:
    s3.download_fileobj('landsat-pds', fn, f)
    
    
    
    
s3 = boto3.client('s3')
with open('FILE_NAME', 'wb') as f:
    s3.download_fileobj('BUCKET_NAME', 'OBJECT_NAME', f)



#
#response = obj.get()
#data = response['Body'].read()
#
#s3.download_file(bucket_name, obj, 'test_bucket.buk')






