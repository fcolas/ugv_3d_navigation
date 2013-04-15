#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from scipy import stats

import sys
import glob
import re
import collections


def dirname2params(dirname):
	name_pattern = r"(?P<dataset>[^_]+)_(?P<test>[^_]+)(?P<params>.*)"
	param_pattern = r"_([a-z]+)([\.\d]+)"
	name_parser = re.compile(name_pattern)
	param_parser = re.compile(param_pattern)
	result = re.match(name_pattern, dirname)
	if result:
		params = re.findall(param_parser, result.groupdict()['params'])
		return dict((n, float(v)) for n, v in params)
	else:
		return {}


def load_path(dirname):
	path = []
	try:
		for line in open(dirname+"/path.csv"):
			path.append([float(t.strip()) for t in line.split(',')])
	except IOError:
		pass
	return path


def compute_metrics(path):
	# x,y,z,nx,ny,nz,dx,dy,dz,posture,edge
	def norm(v):
		return sqrt(np.dot(v, v))
	tot_length = 0.
	sum_of_sines = 0.
	sum_of_1_cos = 0.
	nb_steps = 0
	sum_out_of_plane = 0.
	for i in range(1, len(path)):
		# number of steps
		nb_steps += 1
		# total length
		v = np.array(path[i][:3]) - np.array(path[i-1][:3])
		tot_length += norm(v)
		# change in direction
		d1 = np.array(path[i-1][6:9])
		d2 = np.array(path[i][6:9])
		if (norm(d2) and norm(d1)):
			sum_of_sines += norm(np.cross(d1, d2))/norm(d1)/norm(d2)
			sum_of_1_cos += 1 - 0.5*np.dot(d1, d2)/norm(d1)/norm(d2)
		# out of plane motion
		n1 = np.array(path[i-1][3:6])
		n2 = np.array(path[i][3:6])
		n = .5*(n1+n2)
		if norm(d1):
			sum_out_of_plane += norm(np.dot(d1, n))/norm(d1)/norm(n)
	return nb_steps, tot_length, sum_of_sines, sum_of_1_cos, sum_out_of_plane
		
def paths_metrics_test(respath="test/st2_test_*", filename="metrics.csv"):
	if filename:
		f = open(filename, 'w')
		f.write("min_dist,sigma,sigma_factor,min_sal,nb_steps,tot_length,sines,1_cos,out_of_plane\n")
	data = []
	for dirname in glob.glob(respath):
		#print dirname
		path = load_path(dirname)
		metrics = compute_metrics(path)
		params = dirname2params(dirname)
		if filename:
			f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f\n"%((params['df'], params['s'],
				params['sf'], params['ms'])+metrics))
		data.append((params['df'], params['s'], params['sf'], params['ms'])+metrics)
	if filename:
		f.close()
	return data

def paths_metrics_cost(respath="test/st2_cost_*", filename="metrics.csv"):
	if filename:
		f = open(filename, 'w')
		f.write("saliency_factor,orientation_factor,distance_factor,heading_factor,nb_steps,tot_length,sines,1_cos,out_of_plane\n")
	data = []
	for dirname in glob.glob(respath):
		#print dirname
		path = load_path(dirname)
		metrics = compute_metrics(path)
		params = dirname2params(dirname)
		if filename:
			f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f\n"%((params['s'], params['o'],
				params['d'], params['h'])+metrics))
		data.append((params['s'], params['o'], params['d'], params['h'])+metrics)
	if filename:
		f.close()
	return data


def paths_metrics_sal(respath="test/st2_sal_*", filename="metrics.csv"):
	if filename:
		f = open(filename, 'w')
		f.write("min_saliency,saliency_threshold,maximal_saliency,nb_steps,tot_length,sines,1_cos,out_of_plane\n")
	data = []
	for dirname in glob.glob(respath):
		#print dirname
		path = load_path(dirname)
		metrics = compute_metrics(path)
		params = dirname2params(dirname)
		if filename:
			f.write("%f,%f,%f,%f,%f,%f,%f,%f\n"%((params['min'], params['th'],
				params['max'])+metrics))
		data.append((params['min'], params['th'], params['max'], 0)+metrics)
	if filename:
		f.close()
	return data

def filter_zero(data):
	return [d for d in data if d[4]]

def metrics_correlation(data, indices=(4,5,6,7,8), names=("nb_steps", "tot_length",
		"sines", "1_cos", "out_of_plane")):
	n = 0
	subplot_arg = 250; # FIXME compute from len(indices)
	m = []
	for i in indices:
		m.append(zip(*data)[i])
	for ii, i in enumerate(indices):
		for jj, j in enumerate(indices[ii+1:]):
			n += 1
			plt.subplot(subplot_arg+n)
			plt.plot(m[ii], m[jj+ii+1], '.')
			plt.title(names[jj+ii+1]+" vs "+names[ii])
	plt.show()

def compare(data, param_index, metric_index):
	nd = collections.defaultdict(list)
	for datum in data:
		p = datum[param_index]
		m = datum[metric_index]
		nd[p].append(m)
	param_values = sorted(nd.keys())
	different = False
	for i, v in enumerate(param_values):
		for j in range(i+1, len(param_values)):
			u, p = stats.ranksums(nd[v], nd[param_values[j]])
			if p<0.05:
				#print v, "vs", param_values[j], ":", p
				different = True
			else:
				print v, "vs", param_values[j], ":", p
	return different


def param_eval(data, metric_index=8, names=("min_dist", "sigma", "sigma_factor",
			"min_sal")):
	params = zip(*data)[:4]
	value = zip(*data)[metric_index]
	differences = []
	for i, (param, name) in enumerate(zip(params, names)):
		plt.subplot(221+i)
		plt.plot(param, value, '.')
		#plt.xlim([0., 10.])
		print name, ":"
		if compare(data, i, metric_index):
			differences.append((name, metric_index))
		plt.title(name)
	print differences
	plt.show()
	return differences


def main(series="test"):
	if series=="test":
		data = paths_metrics_test()
		names = ("min_dist", "sigma", "sigma_factor", "min_sal")
	elif series=="cost":
		data = paths_metrics_cost()
		names = ("saliency", "orientation", "distance", "heading")
	elif series=="cost2":
		data = paths_metrics_cost("test/st7_cost2_*")
		names = ("saliency", "orientation", "distance", "heading")
	elif series=="sal":
		data = paths_metrics_sal()
		names = ("min_saliency", "saliency_threshold", "maximal_saliency")
	else:
		print "Unknown data series:", series
	diff = []
	for m in range(4, 9):
		diff.extend(param_eval(data, m, names))
	print diff
	

if __name__=='__main__':
	main(sys.argv[1])
