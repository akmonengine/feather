package feather

import "sync"

func task[T any](workersCount int, data []T, fn func(data T)) {
	var wg sync.WaitGroup
	dataSize := len(data)
	chunkSize := (dataSize + workersCount - 1) / workersCount

	for workerID := 0; workerID < workersCount; workerID++ {
		wg.Add(1)
		go func(start, end int) {
			defer wg.Done()
			for i := start; i < end; i++ {
				fn(data[i])
			}
		}(workerID*chunkSize, min((workerID+1)*chunkSize, dataSize))
	}
	wg.Wait()
}
