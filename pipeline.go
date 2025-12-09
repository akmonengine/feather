package feather

import "sync"

func task(workersCount int, dataSize int, fn func(start, end int)) {
	var wg sync.WaitGroup
	chunkSize := (dataSize + workersCount - 1) / workersCount

	for workerID := 0; workerID < workersCount; workerID++ {
		wg.Add(1)
		go func(start, end int) {
			defer wg.Done()
			fn(start, end)
		}(workerID*chunkSize, min((workerID+1)*chunkSize, dataSize))
	}
	wg.Wait()
}
