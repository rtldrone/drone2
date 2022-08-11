<script>
    import { sendStopCommand } from "./stores"
</script>


<button class="btn btn-error w-[300px] h-[250px] fixed text-6xl text-white" on:click={sendStopCommand} >Stop</button>
