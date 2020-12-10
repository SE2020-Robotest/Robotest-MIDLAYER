#!/usr/bin/env python

from itertools import count
import heapq


def _my_siftdown(heap, startpos, pos):
	newitem = heap[pos]
	while pos > startpos:
		parentpos = (pos - 1) >> 1
		parent = heap[parentpos]
		if heapq.cmp_lt(newitem, parent):
			heap[pos] = parent
			heap[pos][-1] = pos
			pos = parentpos
			continue
		break
	heap[pos] = newitem
	heap[pos][-1] = pos

def _my_siftup(heap, pos):
	endpos = len(heap)
	newitem = heap[pos]
	# Bubble up the smaller child until hitting a leaf.
	childpos = 2*pos + 1    # leftmost child position
	while childpos < endpos:
		# Set childpos to index of smaller child.
		rightpos = childpos + 1
		if rightpos < endpos and not heapq.cmp_lt(heap[childpos], heap[rightpos]):
			childpos = rightpos
		if not heapq.cmp_lt(heap[childpos], newitem):
			break
		# Move the smaller child up.
		heap[pos] = heap[childpos]
		heap[pos][-1] = pos
		pos = childpos
		childpos = 2*pos + 1
	# The leaf at pos is empty now.  Put newitem there, and bubble it up
	# to its final resting place (by sifting its parents down).
	heap[pos] = newitem
	heap[pos][-1] = pos

class my_pq:
	def __init__(self):
		self.pq = []                         	# list of entries arranged in a heap
		self.entry_finder = {}               	# mapping of tasks to entries
		self.counter = count()     		# unique sequence count
		return

	def empty(self):
		return len(self.pq) == 0

	def put(self, data, priority = 0):
		'Add a new task or update the priority of an existing task'
		if data in self.entry_finder:
			self.update(data, priority)
			return
		count = next(self.counter)
		pos = len(self.pq)
		entry = [priority, count, data, pos]
		self.entry_finder[data] = entry
		self.pq.append(entry)
		_my_siftdown(self.pq, 0, len(self.pq) - 1)

	def update(self, data, priority):
		entry = self.entry_finder[data]
		if priority < entry[0]:
			entry[0] = priority
			_my_siftdown(self.pq, 0, entry[-1])
		elif priority > entry[0]:
			entry[0] = priority
			_my_siftup(self.pq, entry[-1])

	def get(self):
		lastelt = self.pq.pop()    # raises appropriate IndexError if heap is empty
		if self.pq:
			returnitem = self.pq[0]
			self.pq[0] = lastelt
			_my_siftup(self.pq, 0)
		else:
			returnitem = lastelt
		return returnitem[2]

def test_my_pq():
	pq = my_pq()
	pq.put('a')
	pq.put('b', 123)
	pq.put('c', 2)
	pq.put('d', 43)
	pq.put('e', 3)
	pq.put('f', 99)
	pq.put('g', 5)
	pq.put('h', 4)
	assert not pq.empty(), "The method 'empty' test failed!"
	pq.put('a', 98)
	pq.put('b', 23)
	pq.put('c', 21)
	pq.put('d', 1)
	pq.update('e', 2)
	pq.update('f', 5)
	pq.update('g', 3)
	pq.update('h', 3)
	correct_ans = ['d', 'e', 'g', 'h', 'f', 'c', 'b', 'a']
	correct_ans.reverse()
	while not pq.empty():
		assert pq.get() == correct_ans.pop(), "Cannot correctly output by priority or cannot correctly update the priority!"


if __name__ == '__main__':
	try:
		test_my_pq()
		print "Test pass!"
	except Exception as e:
		print e
