import {createInterface} from 'node:readline';
import {pathToFileURL} from 'node:url';
const {Lobster} = await import(pathToFileURL(process.argv[2]).href);
for await (const line of createInterface({input: process.stdin, crlfDelay: Infinity})) {
  try {
    const req = JSON.parse(line);
    const call = async (name, state) => {
      const response = await fetch(`${req.url}/${name}`, {
        method: 'POST', headers: {'Content-Type': 'application/json', Authorization: req.token},
        body: JSON.stringify(state), signal: AbortSignal.timeout(180000),
      });
      const value = await response.json();
      if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
      return value;
    };
    let state = req.state;
    do {
      const result = await new Lobster()
        .pipe(async items => [await call('plan', items[0])])
        .pipe(async items => [await call('execute', items[0])])
        .run([state]);
      if (!result.ok) throw new Error(JSON.stringify(result.error));
      state = result.output[0];
    } while (!state.done);
    process.stdout.write(JSON.stringify({ok:true,state})+'\n');
  } catch (error) {
    process.stdout.write(JSON.stringify({ok:false,error:String(error)})+'\n');
  }
}
