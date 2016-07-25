#! /usr/bin/env python
import openravepy as orpy

def remove_objects(env, objects):
  for name in objects:
    body =  env.GetKinBody(name)
    if body is not None:
      env.Remove(body)
