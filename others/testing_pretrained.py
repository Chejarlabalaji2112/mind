#!/usr/bin/env python
# coding: utf-8

# In[1]:


import warnings
warnings.filterwarnings('ignore')


# In[3]:


from tensorflow.keras.models import load_model
import pickle
import librosa
import numpy as np


# In[4]:


def zcr(data,frame_length,hop_length):
    zcr=librosa.feature.zero_crossing_rate(data,frame_length=frame_length,hop_length=hop_length)
    return np.squeeze(zcr)
def rmse(data,frame_length=2048,hop_length=512):
    rmse=librosa.feature.rms(y=data,frame_length=frame_length,hop_length=hop_length)
    return np.squeeze(rmse)
def mfcc(data,sr,frame_length=2048,hop_length=512,flatten:bool=True):
    mfcc=librosa.feature.mfcc(y=data,sr=sr)
    return np.squeeze(mfcc.T)if not flatten else np.ravel(mfcc.T)

def extract_features(data,sr=22050,frame_length=2048,hop_length=512):
    result=np.array([])
    
    result=np.hstack((result,
                      zcr(data,frame_length,hop_length),
                      rmse(data,frame_length,hop_length),
                      mfcc(data,sr,frame_length,hop_length)
                     ))
    return result


# In[11]:


loaded_model = load_model('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/total_best_model.keras')


# In[6]:


with open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/new_scaler2.pickle', 'rb') as f:
    new_scaler2 = pickle.load(f)


with open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/new_encoder2.pickle', 'rb') as f:
    new_encoder2 = pickle.load(f)


# In[7]:


def get_predict_feat(path):
    #d, s_rate= librosa.load(path, duration=2.5, offset=0.6)
    d, s_rate= load_fixed_audio(path)
    #librosa.load(path, duration=2.5, offset=0.6)
    res=extract_features(d)
    result=np.array(res)
    result=np.reshape(result,newshape=(1,2376))
    i_result = new_scaler2.transform(result)
    final_result=np.expand_dims(i_result, axis=2)
    
    return final_result


# In[8]:


emotions1={1:'Neutral', 2:'Calm', 3:'Happy', 4:'Sad', 5:'Angry', 6:'Fear', 7:'Disgust',8:'Surprise'}
def prediction(path1):
    res=get_predict_feat(path1)
    predictions=loaded_model.predict(res)
    y_pred = new_encoder2.inverse_transform(predictions)
    print(y_pred[0][0]) 


# In[9]:


def load_fixed_audio(path, target_duration=2.5):
    y, sr = librosa.load(path,duration=target_duration, offset=0.6)
    if len(y) < target_duration * sr:
        pad_length = int(target_duration * sr) - len(y)
        y = np.pad(y, (0, pad_length))  # Pad with zeros
    return y, sr


# In[12]:


prediction('/home/badri/mine/ser/datasets/test_audio/shock.wav')


# In[1]:


get_ipython().system('jupyter nbconvert --to python testing_pretrained.ipynb')


# import pickle
# from tensorflow.keras.models import Sequential, model_from_json
# 
# 
# import librosa
# import numpy as np
# 
# 
# 
# json_file = open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/CNN_model.json', 'r')
# loaded_model_json = json_file.read()
# json_file.close()
# loaded_model = model_from_json(loaded_model_json, custom_objects={"Sequential": Sequential})
# print("Model loaded successfully from JSON")
# 
# loaded_model.summary()
# 
# loaded_model.load_weights('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/best_model1_weights.h5')
# print("loaded succesfully")
# 
# loaded_model.summary()
# 
# loaded_model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
# 
# 
# type(encoder2)
# 
# prediction("/home/badri/mine/ser/datasets/revdess/Actor_21/03-01-04-02-02-02-21.wav")
# 
# 
# import pandas
# emotions = pandas.read_csv('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/y_label.csv')
# 
# 
# Y=emotions['Emotions'].values
# 
# from sklearn.preprocessing import OneHotEncodermimport numpy as np
# 
# encoder = OneHotEncoder(sparse_output=False)  # Use sparse_output instead of sparse
# Y = encoder.fit_transform(np.array(Y).reshape(-1, 1))
# 
# 
# with open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/new_encoder2.pickle', 'wb') as f:
#     pickle.dump(encoder, f)
# 
# with open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/new_encoder2.pickle', 'rb') as f:
#     new_encoder2 = pickle.load(f)
# 
# 
# 
# prediction('/home/badri/mine/ser/datasets/test_audio/shock.wav')
# 
# loaded_model.save('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/total_best_model.keras')
# 
# model.summary()
# 
# from sklearn.preprocessing import StandardScaler
# new_scaler2 = StandardScaler()
# 
# new_scaler2.mean_ = old_scaler.mean_
# new_scaler2.scale_ = old_scaler.scale_
# new_scaler2.var_ = old_scaler.var_  # If available in the old version
# new_scaler2.n_features_in_ = old_scaler.n_features_in_
# 
# import pickle
# 
# with open('/home/badri/mine/ser/saved_models/kaggle_saved_gpu3hrs/new_scaler2.pickle', 'wb') as f:
#     pickle.dump(new_scaler2, f)
